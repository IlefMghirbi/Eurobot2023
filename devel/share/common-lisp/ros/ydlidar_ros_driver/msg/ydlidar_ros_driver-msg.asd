
(cl:in-package :asdf)

(defsystem "ydlidar_ros_driver-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "EmergencyStop" :depends-on ("_package_EmergencyStop"))
    (:file "_package_EmergencyStop" :depends-on ("_package"))
    (:file "LidarCoord" :depends-on ("_package_LidarCoord"))
    (:file "_package_LidarCoord" :depends-on ("_package"))
  ))