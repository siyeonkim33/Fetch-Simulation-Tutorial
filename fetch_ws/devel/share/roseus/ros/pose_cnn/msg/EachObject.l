;; Auto-generated. Do not edit!


(when (boundp 'pose_cnn::EachObject)
  (if (not (find-package "POSE_CNN"))
    (make-package "POSE_CNN"))
  (shadow 'EachObject (find-package "POSE_CNN")))
(unless (find-package "POSE_CNN::EACHOBJECT")
  (make-package "POSE_CNN::EACHOBJECT"))

(in-package "ROS")
;;//! \htmlinclude EachObject.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass pose_cnn::EachObject
  :super ros::object
  :slots (_name _posestamped ))

(defmethod pose_cnn::EachObject
  (:init
   (&key
    ((:name __name) "")
    ((:posestamped __posestamped) (instance geometry_msgs::PoseStamped :init))
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _posestamped __posestamped)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:posestamped
   (&rest __posestamped)
   (if (keywordp (car __posestamped))
       (send* _posestamped __posestamped)
     (progn
       (if __posestamped (setq _posestamped (car __posestamped)))
       _posestamped)))
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; geometry_msgs/PoseStamped _posestamped
    (send _posestamped :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; geometry_msgs/PoseStamped _posestamped
       (send _posestamped :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/PoseStamped _posestamped
     (send _posestamped :deserialize buf ptr-) (incf ptr- (send _posestamped :serialization-length))
   ;;
   self)
  )

(setf (get pose_cnn::EachObject :md5sum-) "81f443669f369cfa0946fd2ef8ca5011")
(setf (get pose_cnn::EachObject :datatype-) "pose_cnn/EachObject")
(setf (get pose_cnn::EachObject :definition-)
      "###########################################################
# This message describes an object.

# An object might have a name
string name

# Poses of each object
geometry_msgs/PoseStamped posestamped
================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :pose_cnn/EachObject "81f443669f369cfa0946fd2ef8ca5011")


