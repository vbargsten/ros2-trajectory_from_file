# trajectory_from_file
  
This node reads a text file with comma-separated values (CSV) and publishes  
the data on a ROS2 topic. Currently, only joint position data are implemented.  
The file must contain a header. The node currently only supports fixed sampling 
rate and does not use the time column. 

## File format
However, the reader (`CSVPublisher` class)
already interprets the following header fields:  
  
- for the time column:
	- one or none time columns can be included
    - Must match the regex `"^((t|T)ime|t\\[s\\]|t|T)$"`  
    - examples: `time`, `t[s]`, `t`, `T`

- for joint positions
	- is mandatory
	- one or more
	- supported formats:
		- `q0`, `q1`, ...
		- `q1`, `q2`, ...
		- `q[0]`, `q[1]`, ...
		- `q_<name>`, where `<name>`can be any alpha-numeric name including '-' or '_', but without spaces, not starting or ending with `-`or `_`
		- `q[<name>]`
- for joint velocities
	- is optional
	- supported formats:
		- must follow the format used for `q` with
		- prefix `dq`or `qd`
- for joint accelerations
	- is optional
	- supported formats:
		- must follow the format used for `q` with
		- prefix `ddq`if `dq`is used or `qdd` if `qd`is used for velocities
- for joint torques/forces
	- is optional
	- supported formats:
		- must follow the format used for `q` with
		- prefix `tau` or `Tau`

The actual values must be formatted in floating point format 
with `.`as decimal separator, and `,`as separator of the values.

The currently supported message types:

- `std_msgs::msg::Float64MultiArray`
	- the order will follow the ordering of the joint positions columns in the header or the supplied joint names (not implemented, will implement on request :))
	

## Parameters:
- Parameter name: input_file  
    - Type: string  
    - Description: Path to the input CSV file  
- Parameter name: output_topic_name  
    - Type: string  
    - Description: The name of the topic the data will be published on. Default "/trajectory_from_file/joint_command".  
- Parameter name: sample_period  
    - Type: double  
    - Description: Time in seconds between outputting the samples.  
- Parameter name: speed_factor  
    - Type: double  
    - Description: Set a value between 0 and 1 so reduce output frequency. Set value >1 to increase output speed.  


