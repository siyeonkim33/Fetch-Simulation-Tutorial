
(cl:in-package :asdf)

(defsystem "pose_cnn-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Class" :depends-on ("_package_Class"))
    (:file "_package_Class" :depends-on ("_package"))
    (:file "EachObject" :depends-on ("_package_EachObject"))
    (:file "_package_EachObject" :depends-on ("_package"))
  ))