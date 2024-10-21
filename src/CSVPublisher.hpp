#pragma once

#include <string>
#include <fstream>
#include <regex>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace trajectory_from_file
{

    static std::string TIME_COLUMN_REGEX = "^((t|T)ime|t\\[s\\]|t|T)$";
    
class CSVPublisherInterface {
    public:
        virtual void update(){ };
        virtual bool isFinished() = 0;
        virtual bool readCSV(std::string filename) = 0;
        virtual void reset() = 0;
};

template<class T> class CSVPublisher : public CSVPublisherInterface {

    protected:

        std::vector<std::string> joint_names_;
        std::map<std::pair<std::string, std::string>, int> mapping_;
        std::weak_ptr<rclcpp::Node> task_context_;
        std::shared_ptr<rclcpp::Publisher<T>> out_port_;
        T out_;
        std::vector<std::string> column_names_;
        std::map<std::string, int> name_to_index_;
        std::vector<std::vector< double >> data_;
        uint currentIndex_ = 0;
        int timeColumnIndex_ = -1;

    public:
        CSVPublisher(){ }
        CSVPublisher(std::string filename,
                  std::string port_name,
                  std::shared_ptr<rclcpp::Node> node){
            
            if (!readCSV(filename)) {
                throw std::runtime_error("Reading CSV file \"" + filename + "\" failed. Stopping.");
            }
            
            out_port_ =  node->create_publisher<T>(port_name, 1);
            task_context_ = node;            


            // // strip .csv from filename
            // std::string port_name_ = filename.substr(
            //     0, filename.find_last_of("."));            
            
            
            // there is no such concept of ports in ROS. a node can store its publishers anywhere
            // could only try to get all topics
            // auto task_context_->get_topic_names_and_types();
            // std::map<std::string, std::vector<std::string> > rclcpp::node::Node::get_topic_names_and_types 	( 		) 	const
            // if (task_context_->ports()->getPort(port_name) != NULL){
            //    throw std::runtime_error(
            //        std::string("Port already present: ") + port_name);
            // }
            // task_context_->ports()->addPort(port_name, *out_port_);

        }
        ~CSVPublisher(){
            // publisher should be destroyed automatically
        }

        void reset() override{
            currentIndex_ = 0;
        }

        bool isFinished() override{
            return currentIndex_ == data_.size();
        }

        bool readCSV(std::string filename) override{
            std::fstream fp;
            fp.open(filename, std::iostream::ios_base::in);
            if (!fp)
            {
                std::cerr << "Trajectory_from_file: Could not open file\n";
                return false;
            }
            std::string buffer;
            std::vector<std::string> strings;
            std::vector<double > rowValues;

            bool isHeader = true;

            // Header
            while (std::getline(fp, buffer))
            {
                strings.clear();
                splitString(buffer, ',', strings);

                unsigned int index = 0;
                for (std::vector< std::string >::iterator it = strings.begin();
                        it != strings.end(); ++it)
                {
                    boost::algorithm::trim(*it);
                    if (isHeader)
                    {
                        if (name_to_index_.find(*it) != name_to_index_.end()) {
                            std::cerr << __FILE__ << ", " << __LINE__ << ": Column names are not unique (column \"" << *it << "\").\n";
                            return false;  
                        }
                        name_to_index_[*it] = index++;
                        column_names_.push_back( *it ); 
                    }
                    else
                    {
                        rowValues.push_back(boost::lexical_cast< double >(*it));
                    }

                }

                if (!isHeader)
                {
                    
                    if (name_to_index_.size() != rowValues.size()) {
                        std::cerr << __FILE__ << ", " << __LINE__ << ": Number of columns does not match at line " << data_.size()+1 << "\n";
                        return false;   
                    }
                    
                    data_.push_back(rowValues);
                    rowValues.clear();
                }
                else
                {
                    isHeader = false;
                }
            }

            std::cout << "CSV-file: #Columns = " << name_to_index_.size()
                << ", #rows = " << data_.size() << std::endl;
            fp.close();

            return true;
        }

        void splitString(
                std::string str,
                char delim,
                std::vector< std::string >& strings){
            strings.clear();

            std::size_t delimPos = 0;
            std::size_t startPos = 0;

            while (delimPos != std::string::npos)
            {
                delimPos = str.find(delim, startPos);
                if (delimPos == std::string::npos)
                {
                    strings.push_back(str.substr(startPos, str.length()-startPos));
                }
                else
                {
                    strings.push_back(str.substr(startPos, delimPos - startPos));
                }

                startPos = delimPos+1;
            }
        }

};

class JointPositionsFromCSVPublisher : public CSVPublisher<std_msgs::msg::Float64MultiArray> {
    friend class CSVPublisher<std_msgs::msg::Float64MultiArray>;

    private:
        std::vector<int> posColumnIndices_;
        std::vector<int> velColumnIndices_;
        std::vector<int> accColumnIndices_;
        std::vector<int> tauColumnIndices_;
    
    public:
        JointPositionsFromCSVPublisher(
            std::string filename,
            std::string port_name,
            std::shared_ptr<rclcpp::Node> node,
            std::vector<std::string> joint_names
        ) : CSVPublisher<std_msgs::msg::Float64MultiArray> { filename, port_name, node } {
            joint_names_ = joint_names;
            
            // col name can be
            // 1) q_<joint_name>, qd_.., qdd_..., Tau_...
            // 2) q(_)<index>, dq.., ddq..., tau..
            // 3) q(_)<index+1>, dq.., ddq..., tau..
            if (!mapColumns()) {
                throw std::runtime_error("Failed to map columns. Stopping.");
            }
            
        }

        bool mapColumns() {
            // cols can only be mapped once, so collect them separately
            auto remaining_name_to_index = name_to_index_;
            auto remaining_column_names = column_names_;
            
            // search for a time column
            for (auto& item : name_to_index_){
                if ( std::regex_match(item.first, std::regex(TIME_COLUMN_REGEX)) ) {
                    
                    if (timeColumnIndex_ >= 0) {
                        std::cerr << __FILE__ << ", " << __LINE__ << ": I found multiple time columns. Stopping." << std::endl;
                        return false;
                    }
                    
                    timeColumnIndex_ = item.second;
                    std::cout << "time index" << timeColumnIndex_ << std::endl;
                    
                    remaining_column_names.erase(std::find(remaining_column_names.begin(),remaining_column_names.end(), item.first));
                    remaining_name_to_index.erase(item.first);
                    
                    continue;
                } 
            }
            
                
            std::string delimiter = "_";
            bool nameInBrackets = false;
                
            if (joint_names_.size() > 0) {
                // names are provided
                bool all_numbered_names = std::all_of(
                    joint_names_.cbegin(), joint_names_.cend(), 
                    [] (std::string s) {  
                        return std::regex_match(s, std::regex("^[0-9]+"));
                    }
                );
                
                if (remaining_name_to_index.find( "q_" + joint_names_[0] ) != remaining_name_to_index.end()) {
                    delimiter = "_";
                    nameInBrackets = false;
                } else if (remaining_name_to_index.find( "q[" + joint_names_[0] + "]" ) != remaining_name_to_index.end()) {
                    delimiter = "";
                    nameInBrackets = true;
                } else if (all_numbered_names && remaining_name_to_index.find( "q"+ joint_names_[0] ) != remaining_name_to_index.end()) {
                    delimiter = "";
                    nameInBrackets = false;
                } 
                
            } else {
                // no joint names are given
                // we need to find all the columns / names
                // order might be important, so not using the map here
                // 
                
                /*
                 * Position must be included, the other fields are optional.
                 * 
                 * The position field name options we are supporting:
                 * 
                 * - zero based indexing
                 *   q_0
                 *   q0
                 *   q[0]
                 *
                 * - 1-based indexing, no candidate from above
                 *   q_1
                 *   q1
                 *
                 * - name based, use order as index
                 *   q_<name>
                 *   q[<name>]
                 *   not supported: q<name>
                 */
                
                // find out if we got named fields or which flavor of numbered fields
                
                bool matchByIdx = false;
                bool zeroBasedIdx = true;
                
                if (remaining_name_to_index.find( "q0" ) != remaining_name_to_index.end()) {
                    delimiter = "";
                    matchByIdx = true;
                    zeroBasedIdx = true;
                    nameInBrackets = false;
                } else if (remaining_name_to_index.find( "q_0" ) != remaining_name_to_index.end()) {
                    delimiter = "_";
                    matchByIdx = true;
                    zeroBasedIdx = true;
                    nameInBrackets = false;
                } else if (remaining_name_to_index.find( "q[0]" ) != remaining_name_to_index.end()) {
                    delimiter = "";
                    matchByIdx = true;
                    zeroBasedIdx = true;
                    nameInBrackets = true;
                } else if (remaining_name_to_index.find( "q1" ) != remaining_name_to_index.end()) {
                    delimiter = "";
                    matchByIdx = true;
                    zeroBasedIdx = false;
                    nameInBrackets = false;
                } else if (remaining_name_to_index.find( "q_1" ) != remaining_name_to_index.end()) {
                    delimiter = "_";
                    matchByIdx = true;
                    zeroBasedIdx = false;
                    nameInBrackets = false;
                }
                
                
                if (matchByIdx) {
                    // numbered fields it is
                    
                    int candidateIdx = zeroBasedIdx ? 0 : 1;
                    
                    while (true) {
                        auto candidate = "q" + delimiter + std::to_string(candidateIdx);
                        
                        if (nameInBrackets) {                            
                            candidate = "q[" + std::to_string(candidateIdx) + "]";
                        }
                        
                        if ( remaining_name_to_index.find(candidate) == remaining_name_to_index.end() ) {
                            break;
                        }
                        
                        joint_names_.push_back(std::to_string(candidateIdx));
                        
                        candidateIdx++;
                    }
                    
                } else {
                    // a named position field it is...maybe
                    size_t idx_first_q_col = remaining_column_names.size();
                    
                    // find the first column starting with q
                    for (size_t idx=0; idx<remaining_column_names.size(); idx++) {
                        if (remaining_column_names[idx].rfind("q", 0) == 0) {
                            // column name does start with q
                            idx_first_q_col = idx;
                            break;
                        }
                    }
                    
                    if (idx_first_q_col >= remaining_column_names.size()) {
                        std::cerr << __FILE__ << ", " << __LINE__ << ": I could not find any column name starting with \"q\". Stopping." << std::endl;
                        return false;                        
                    }
                    
                    
                    delimiter = "";
                    nameInBrackets = true;
                    
                    // find the delimiter/bracket. only q_<name> or q[<name>] is allowed
                    if (remaining_column_names[idx_first_q_col].rfind("q_", 0) == 0) {
                        delimiter = "_";
                        nameInBrackets = false;
                    } else if (!std::regex_match(remaining_column_names[idx_first_q_col], std::regex("^q\\[([0-9a-zA-Z]+([0-9a-zA-Z_\\-/\\.]+[0-9a-zA-Z])?)\\]$"))) {
                        std::cerr << __FILE__ << ", " << __LINE__ << ": I do not understand the \"q\" column name format. Stopping." << std::endl;
                        return false;
                    }
                        
                    auto available_cols = remaining_column_names;
                    
                    // collect the names in the order of the header
                    for (size_t idx=0; idx<available_cols.size(); idx++) {
                        auto name_regex = std::regex("^q" + delimiter + R"~~~#~~~(([0-9a-zA-Z]+([0-9a-zA-Z_\-\/\.]+[0-9a-zA-Z])?)$)~~~#~~~");
                        
                        if (nameInBrackets) {
                            name_regex = std::regex(R"~~~#~~~(^q\[([0-9a-zA-Z]+([0-9a-zA-Z_\-\/\.]+[0-9a-zA-Z])?)\]$)~~~#~~~");
                        }
                        
                        std::smatch name_matches;
                    
                        if (std::regex_match(available_cols[idx], name_matches, name_regex)) {                            
                            joint_names_.push_back( name_matches[1] );
                            //remaining_column_names.remove( remaining_column_names[idx] );
                            //remaining_name_to_index.erase( remaining_column_names[idx] );
                        }
                    }                    
                }                
            }
            
            
            if (joint_names_.size() == 0) {
                std::cerr << __FILE__ << ", " << __LINE__ << ": I could not find any column with indexing for \"q\". Stopping." << std::endl;
                return false;                        
            }
                            
                
            posColumnIndices_.clear();
            velColumnIndices_.clear();
            accColumnIndices_.clear();
            tauColumnIndices_.clear();
                
            posColumnIndices_.resize(joint_names_.size(), -1);
            velColumnIndices_.resize(joint_names_.size(), -1);
            accColumnIndices_.resize(joint_names_.size(), -1);
            tauColumnIndices_.resize(joint_names_.size(), -1);    
            
            // qd or dq
            std::vector<std::string> pos_prefix_candidates = {"q"};
            std::vector<std::string> vel_prefix_candidates = {"dq", "qd"};
            std::vector<std::string> acc_prefix_candidates = {"ddq", "qdd"};
            std::vector<std::string> tau_prefix_candidates = {"tau", "Tau"};                    
                                       
            // iterate over all pos candidates    
            for (size_t idx_p=0; idx_p<pos_prefix_candidates.size(); idx_p++) {
                
                bool pos_prefix_matches = false;
                
                for (size_t idx_j=0; idx_j<joint_names_.size(); idx_j++) {
                    
                    std::string candidate = pos_prefix_candidates[idx_p] + delimiter + joint_names_[idx_j];
                    
                    if (nameInBrackets) {                            
                        candidate = pos_prefix_candidates[idx_p] + "[" + joint_names_[idx_j] + "]";
                    }    
                    
                    if (remaining_name_to_index.find( candidate ) != remaining_name_to_index.end()) {
                        posColumnIndices_[idx_j] = remaining_name_to_index[ candidate ];                        
                        remaining_column_names.erase(std::find(remaining_column_names.begin(),remaining_column_names.end(), candidate));
                        remaining_name_to_index.erase(candidate);
                        pos_prefix_matches = true;
                    }/* else {
                        break;
                    }*/
                }
                        
                if (pos_prefix_matches) {
                    //pos_prefix = pos_prefix_candidates[idx_p];
                    break;
                }
            }
                                       
            // iterate over all vel candidates    
            for (size_t idx_p=0; idx_p<vel_prefix_candidates.size(); idx_p++) {
                
                bool vel_prefix_matches = false;
                
                for (size_t idx_j=0; idx_j<joint_names_.size(); idx_j++) {
                    
                    std::string candidate = vel_prefix_candidates[idx_p] + delimiter + joint_names_[idx_j];
                    
                    if (nameInBrackets) {                            
                        candidate = vel_prefix_candidates[idx_p] + "[" + candidate + "]";
                    }    
                    
                    if (remaining_name_to_index.find( candidate ) != remaining_name_to_index.end()) {
                        velColumnIndices_[idx_j] = remaining_name_to_index[ candidate ];
                        remaining_column_names.erase(std::find(remaining_column_names.begin(),remaining_column_names.end(), candidate));
                        remaining_name_to_index.erase(candidate);
                        vel_prefix_matches = true;
                    }
                }
                        
                if (vel_prefix_matches) {
                    //vel_prefix = vel_prefix_candidates[idx_p];
                    // the acc prefix must follow the same flavor (qdd or ddq)
                    auto tmp = acc_prefix_candidates[idx_p];
                    acc_prefix_candidates.clear();
                    acc_prefix_candidates.push_back(tmp);
                    break;
                }
            }
                        
            // iterate over all acc candidates   
            for (size_t idx_p=0; idx_p<acc_prefix_candidates.size(); idx_p++) {
                
                bool acc_prefix_matches = false;
                
                for (size_t idx_j=0; idx_j<joint_names_.size(); idx_j++) {
                    
                    std::string candidate = acc_prefix_candidates[idx_p] + delimiter + joint_names_[idx_j];
                    
                    if (nameInBrackets) {                            
                        candidate = acc_prefix_candidates[idx_p] + "[" + candidate + "]";
                    }    
                    
                    if (remaining_name_to_index.find( candidate ) != remaining_name_to_index.end()) {
                        accColumnIndices_[idx_j] = remaining_name_to_index[ candidate ];
                        remaining_column_names.erase(std::find(remaining_column_names.begin(),remaining_column_names.end(), candidate));
                        remaining_name_to_index.erase(candidate);
                        acc_prefix_matches = true;
                    } else {
                        break;
                    }
                }
                
                if (acc_prefix_matches) {
                    //acc_prefix = acc_prefix_candidates[idx_p];
                    break;
                }                        
            }
                        
            // iterate over all tau candidates  
            for (size_t idx_p=0; idx_p<tau_prefix_candidates.size(); idx_p++) {
                
                bool tau_prefix_matches = false;
                
                for (size_t idx_j=0; idx_j<joint_names_.size(); idx_j++) {
                    
                    std::string candidate = tau_prefix_candidates[idx_p] + delimiter + joint_names_[idx_j];
                    
                    if (nameInBrackets) {                            
                        candidate = acc_prefix_candidates[idx_p] + "[" + candidate + "]";
                    }    
                    
                    if (remaining_name_to_index.find( candidate ) != remaining_name_to_index.end()) {
                        tauColumnIndices_[idx_j] = remaining_name_to_index[candidate];
                        remaining_column_names.erase(std::find(remaining_column_names.begin(),remaining_column_names.end(), candidate));
                        remaining_name_to_index.erase(candidate);
                        tau_prefix_matches = true;
                    } /*else {
                        break;
                    }*/
                }
                
                if (tau_prefix_matches) {
                    //tau_prefix = tau_prefix_candidates[idx_p];
                    break;
                }
            }
            
            std::cout << "The following mapping has been made:\n";
            std::cout << "time->" << (timeColumnIndex_ >= 0 ? column_names_[timeColumnIndex_] : "---") << "\n";
            
            for (size_t i=0; i<joint_names_.size(); i++) {
                std::cout << "J[" << i << "]:\t" 
                    << (posColumnIndices_[i] >= 0 ? column_names_[posColumnIndices_[i]] : "---") << "\t"
                    << (velColumnIndices_[i] >= 0 ? column_names_[velColumnIndices_[i]] : "---") << "\t"
                    << (accColumnIndices_[i] >= 0 ? column_names_[accColumnIndices_[i]] : "---") << "\t"
                    << (tauColumnIndices_[i] >= 0 ? column_names_[tauColumnIndices_[i]] : "---") << "\n";
                
            }
            
            return true;
            
        }
        
        void update() override{
            // base::Time cur_time = base::Time::now();
            if (currentIndex_ >= data_.size()) {
                return;
            }
            out_.data.resize(posColumnIndices_.size());
                
            for (size_t idx_j=0; idx_j<posColumnIndices_.size(); idx_j++) {
                out_.data[idx_j] = posColumnIndices_[idx_j] >=0 ? data_[currentIndex_][posColumnIndices_[idx_j]] : std::numeric_limits<double>::quiet_NaN();
//                 for (auto& entry : mapping_){
//                     std::string joint_name = entry.first.first;
//                     std::string value_type = entry.first.second;
//                     int col = entry.second;
// 
//                     if (value_type == "q"){
//                         out_[joint_name].position = data_.at(currentIndex_).at(col);
//                     } else if (value_type == "qd"){
//                         out_[joint_name].speed = 0.0; //data_.at(currentIndex_).at(col);
//                     } else if (value_type == "qdd"){
//                         out_[joint_name].acceleration = 0.0; //data_.at(currentIndex_).at(col);
//                     } else if (value_type == "Tau"){
//                         out_[joint_name].effort = 0.0; //data_.at(currentIndex_).at(col);
//                     } else {
//                         std::cerr << "Warning: Unknown data type in header: "
//                             << value_type
//                             << " for joint: "
//                             << joint_name
//                             << ". Possible values are q, qd, qdd and Tau."
//                             << " Entry is ignored!"
//                             << std::endl;
//                     }
//                 }
//                 if (timeColumnIndex_ >= 0){
//                     out_.time.microseconds = data_.at(currentIndex_).at(timeColumnIndex_) * 1e6;
//                 }
//                 else {
//                     out_.time = base::Time::now();
//                 }
                }
            currentIndex_++;
            out_port_->publish(out_);
        }

};

// class BaseSamplesWrenchesCSVPublisher : public CSVPublisher<base::samples::Wrenches>{
//     friend class CSVPublisher<base::samples::Wrenches>;
// 
//     public:
//         BaseSamplesWrenchesCSVPublisher(std::string filename,
//                                      std::string port_name,
//                                      RTT::TaskContext* tc)
//         : CSVPublisher<base::samples::Wrenches> { filename, port_name, tc } {
// 
//             readHeaderWithDelimiter();
//             out_.names = joint_names_;
// 
//         }
// 
//         void update() override{
//             if (currentIndex_ < data_.size()){
//                 for (auto& entry : mapping_){
//                     std::string joint_name = entry.first.first;
//                     std::string value_type = entry.first.second;
//                     int col = entry.second;
// 
//                     if (value_type == "Fx"){
//                         out_[joint_name].force[0] = data_.at(currentIndex_).at(col);
//                     } else if (value_type == "Fy"){
//                         out_[joint_name].force[1] = data_.at(currentIndex_).at(col);
//                     } else if (value_type == "Fz"){
//                         out_[joint_name].force[2] = data_.at(currentIndex_).at(col);
//                     } else if (value_type == "Tx"){
//                         out_[joint_name].torque[0] = data_.at(currentIndex_).at(col);
//                     } else if (value_type == "Ty"){
//                         out_[joint_name].torque[1] = data_.at(currentIndex_).at(col);
//                     } else if (value_type == "Tz"){
//                         out_[joint_name].torque[2] = data_.at(currentIndex_).at(col);
//                     } else {
//                         std::cerr << "Warning: Unknown data type in header: "
//                             << value_type
//                             << " for joint: "
//                             << joint_name
//                             << ". Possible values are Fx, Fy, Fz, Tx, Ty, Tz."
//                             << " Entry is ignored!"
//                             << std::endl;
// 
//                     }
//                 }
//                 if (timeColumnIndex_ >= 0){
//                     out_.time.microseconds = data_.at(currentIndex_).at(timeColumnIndex_) * 1e6;
//                 }
//                 else {
//                     out_.time = base::Time::now();
//                 }
//                 currentIndex_++;
//                 out_port_->write(out_);
//             }
//         }
// 
// };
// 
// 
// class BaseSamplesRigidBodyStateCSVPublisher : public CSVPublisher<base::samples::RigidBodyState>{
//     friend class CSVPublisher<base::samples::RigidBodyState>;
// 
//     public:
//         BaseSamplesRigidBodyStateCSVPublisher(std::string filename,
//                                            std::string port_name,
//                                            RTT::TaskContext* tc)
//         : CSVPublisher<base::samples::RigidBodyState> { filename, port_name, tc } { }
// 
//         void update() override{
//             if (currentIndex_ < data_.size()){
//                 out_.time = base::Time::now();
//                 for (auto& entry : name_to_index_){
//                     std::string name = entry.first;
//                     int col = entry.second;
//                     if (name == "sourceFrame"){
//                         out_.sourceFrame = data_.at(currentIndex_).at(col);
//                     } else if (name == "TargetFrame"){
//                         out_.targetFrame = data_.at(currentIndex_).at(col);
//                     }
//                     if (name[0] == 'X'){
//                         out_.position[0] = data_.at(currentIndex_).at(col);
//                     } else if (name[0] == 'Y'){
//                         out_.position[1] = data_.at(currentIndex_).at(col);
//                     } else if (name[0] == 'Z'){
//                         out_.position[2] = data_.at(currentIndex_).at(col);
//                     } else if (name[0] == 'Q'){
//                         switch (name[1]){
//                             case 'x':
//                                 out_.orientation.vec()[0] = data_.at(currentIndex_).at(col);
//                                 break;
//                             case 'y':
//                                 out_.orientation.vec()[1] = data_.at(currentIndex_).at(col);
//                                 break;
//                             case 'z':
//                                 out_.orientation.vec()[2] = data_.at(currentIndex_).at(col);
//                                 break;
//                             case 'w':
//                                 out_.orientation.w() = data_.at(currentIndex_).at(col);
//                                 break;
//                             default:
//                                 std::cerr << "Warning: Unknown data type in header: "
//                                     << name
//                                     << std::endl;
//                         }
//                     } else if (name[0] == 'v'){
//                         switch (name[1]){
//                             case 'x':
//                                 out_.velocity[0] = data_.at(currentIndex_).at(col);
//                                 break;
//                             case 'y':
//                                 out_.velocity[1] = data_.at(currentIndex_).at(col);
//                                 break;
//                             case 'z':
//                                 out_.velocity[2] = data_.at(currentIndex_).at(col);
//                                 break;
//                             default:
//                                 std::cerr << "Warning: Unknown data type in header: "
//                                     << name
//                                     << std::endl;
//                         }
//                     }else if (name[0] == 'w'){
//                         switch (name[1]){
//                             case 'x':
//                                 out_.angular_velocity[0] = data_.at(currentIndex_).at(col);
//                                 break;
//                             case 'y':
//                                 out_.angular_velocity[1] = data_.at(currentIndex_).at(col);
//                                 break;
//                             case 'z':
//                                 out_.angular_velocity[2] = data_.at(currentIndex_).at(col);
//                                 break;
//                             default:
//                                 std::cerr << "Warning: Unknown data type in header: "
//                                     << name
//                                     << std::endl;
//                         }
//                     } else if  (name == "t[s]" or name == "time"){
//                         out_.time.microseconds = data_.at(currentIndex_).at(col) * 1e6;
// 
//                     } else {
//                         std::cerr << "Warning: Unknown data type in header: "
//                             << name
//                             << ". Possible values are Fx, Fy, Fz, Tx, Ty, Tz, t[s] and time"
//                             << " Entry is ignored!"
//                             << std::endl;
// 
//                     }
//                 }
// 
//                 currentIndex_++;
//                 out_port_->write(out_);
//             }
//         }
// 
// };
}
