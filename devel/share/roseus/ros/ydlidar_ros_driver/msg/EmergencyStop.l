;; Auto-generated. Do not edit!


(when (boundp 'ydlidar_ros_driver::EmergencyStop)
  (if (not (find-package "YDLIDAR_ROS_DRIVER"))
    (make-package "YDLIDAR_ROS_DRIVER"))
  (shadow 'EmergencyStop (find-package "YDLIDAR_ROS_DRIVER")))
(unless (find-package "YDLIDAR_ROS_DRIVER::EMERGENCYSTOP")
  (make-package "YDLIDAR_ROS_DRIVER::EMERGENCYSTOP"))

(in-package "ROS")
;;//! \htmlinclude EmergencyStop.msg.html


(defclass ydlidar_ros_driver::EmergencyStop
  :super ros::object
  :slots (_signal ))

(defmethod ydlidar_ros_driver::EmergencyStop
  (:init
   (&key
    ((:signal __signal) "")
    )
   (send-super :init)
   (setq _signal (string __signal))
   self)
  (:signal
   (&optional __signal)
   (if __signal (setq _signal __signal)) _signal)
  (:serialization-length
   ()
   (+
    ;; string _signal
    4 (length _signal)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _signal
       (write-long (length _signal) s) (princ _signal s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _signal
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _signal (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get ydlidar_ros_driver::EmergencyStop :md5sum-) "9591b8ace81feee36c93130ad3fe6a19")
(setf (get ydlidar_ros_driver::EmergencyStop :datatype-) "ydlidar_ros_driver/EmergencyStop")
(setf (get ydlidar_ros_driver::EmergencyStop :definition-)
      "string signal

")



(provide :ydlidar_ros_driver/EmergencyStop "9591b8ace81feee36c93130ad3fe6a19")


