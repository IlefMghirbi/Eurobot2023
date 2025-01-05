;; Auto-generated. Do not edit!


(when (boundp 'master::EmergencyStop)
  (if (not (find-package "MASTER"))
    (make-package "MASTER"))
  (shadow 'EmergencyStop (find-package "MASTER")))
(unless (find-package "MASTER::EMERGENCYSTOP")
  (make-package "MASTER::EMERGENCYSTOP"))

(in-package "ROS")
;;//! \htmlinclude EmergencyStop.msg.html


(defclass master::EmergencyStop
  :super ros::object
  :slots (_signal ))

(defmethod master::EmergencyStop
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

(setf (get master::EmergencyStop :md5sum-) "9591b8ace81feee36c93130ad3fe6a19")
(setf (get master::EmergencyStop :datatype-) "master/EmergencyStop")
(setf (get master::EmergencyStop :definition-)
      "string signal

")



(provide :master/EmergencyStop "9591b8ace81feee36c93130ad3fe6a19")


