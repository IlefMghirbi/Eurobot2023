;; Auto-generated. Do not edit!


(when (boundp 'navigation_pkg::EmergencyStop)
  (if (not (find-package "NAVIGATION_PKG"))
    (make-package "NAVIGATION_PKG"))
  (shadow 'EmergencyStop (find-package "NAVIGATION_PKG")))
(unless (find-package "NAVIGATION_PKG::EMERGENCYSTOP")
  (make-package "NAVIGATION_PKG::EMERGENCYSTOP"))

(in-package "ROS")
;;//! \htmlinclude EmergencyStop.msg.html


(defclass navigation_pkg::EmergencyStop
  :super ros::object
  :slots (_signal ))

(defmethod navigation_pkg::EmergencyStop
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

(setf (get navigation_pkg::EmergencyStop :md5sum-) "9591b8ace81feee36c93130ad3fe6a19")
(setf (get navigation_pkg::EmergencyStop :datatype-) "navigation_pkg/EmergencyStop")
(setf (get navigation_pkg::EmergencyStop :definition-)
      "string signal

")



(provide :navigation_pkg/EmergencyStop "9591b8ace81feee36c93130ad3fe6a19")


