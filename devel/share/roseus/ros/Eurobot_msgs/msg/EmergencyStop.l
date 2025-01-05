;; Auto-generated. Do not edit!


(when (boundp 'Eurobot_msgs::EmergencyStop)
  (if (not (find-package "EUROBOT_MSGS"))
    (make-package "EUROBOT_MSGS"))
  (shadow 'EmergencyStop (find-package "EUROBOT_MSGS")))
(unless (find-package "EUROBOT_MSGS::EMERGENCYSTOP")
  (make-package "EUROBOT_MSGS::EMERGENCYSTOP"))

(in-package "ROS")
;;//! \htmlinclude EmergencyStop.msg.html


(defclass Eurobot_msgs::EmergencyStop
  :super ros::object
  :slots (_signal ))

(defmethod Eurobot_msgs::EmergencyStop
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

(setf (get Eurobot_msgs::EmergencyStop :md5sum-) "9591b8ace81feee36c93130ad3fe6a19")
(setf (get Eurobot_msgs::EmergencyStop :datatype-) "Eurobot_msgs/EmergencyStop")
(setf (get Eurobot_msgs::EmergencyStop :definition-)
      "string signal

")



(provide :Eurobot_msgs/EmergencyStop "9591b8ace81feee36c93130ad3fe6a19")


