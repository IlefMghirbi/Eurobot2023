
(cl:in-package :asdf)

(defsystem "navigation_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "EmergencyStop" :depends-on ("_package_EmergencyStop"))
    (:file "_package_EmergencyStop" :depends-on ("_package"))
    (:file "LidarCoord" :depends-on ("_package_LidarCoord"))
    (:file "_package_LidarCoord" :depends-on ("_package"))
    (:file "RobotCoord" :depends-on ("_package_RobotCoord"))
    (:file "_package_RobotCoord" :depends-on ("_package"))
    (:file "State" :depends-on ("_package_State"))
    (:file "_package_State" :depends-on ("_package"))
  ))