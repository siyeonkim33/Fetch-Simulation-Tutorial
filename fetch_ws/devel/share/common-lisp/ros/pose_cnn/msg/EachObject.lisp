; Auto-generated. Do not edit!


(cl:in-package pose_cnn-msg)


;//! \htmlinclude EachObject.msg.html

(cl:defclass <EachObject> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (posestamped
    :reader posestamped
    :initarg :posestamped
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass EachObject (<EachObject>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EachObject>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EachObject)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pose_cnn-msg:<EachObject> is deprecated: use pose_cnn-msg:EachObject instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <EachObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pose_cnn-msg:name-val is deprecated.  Use pose_cnn-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'posestamped-val :lambda-list '(m))
(cl:defmethod posestamped-val ((m <EachObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pose_cnn-msg:posestamped-val is deprecated.  Use pose_cnn-msg:posestamped instead.")
  (posestamped m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EachObject>) ostream)
  "Serializes a message object of type '<EachObject>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'posestamped) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EachObject>) istream)
  "Deserializes a message object of type '<EachObject>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'posestamped) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EachObject>)))
  "Returns string type for a message object of type '<EachObject>"
  "pose_cnn/EachObject")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EachObject)))
  "Returns string type for a message object of type 'EachObject"
  "pose_cnn/EachObject")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EachObject>)))
  "Returns md5sum for a message object of type '<EachObject>"
  "81f443669f369cfa0946fd2ef8ca5011")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EachObject)))
  "Returns md5sum for a message object of type 'EachObject"
  "81f443669f369cfa0946fd2ef8ca5011")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EachObject>)))
  "Returns full string definition for message of type '<EachObject>"
  (cl:format cl:nil "###########################################################~%# This message describes an object.~%~%# An object might have a name~%string name~%~%# Poses of each object~%geometry_msgs/PoseStamped posestamped~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EachObject)))
  "Returns full string definition for message of type 'EachObject"
  (cl:format cl:nil "###########################################################~%# This message describes an object.~%~%# An object might have a name~%string name~%~%# Poses of each object~%geometry_msgs/PoseStamped posestamped~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EachObject>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'posestamped))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EachObject>))
  "Converts a ROS message object to a list"
  (cl:list 'EachObject
    (cl:cons ':name (name msg))
    (cl:cons ':posestamped (posestamped msg))
))
