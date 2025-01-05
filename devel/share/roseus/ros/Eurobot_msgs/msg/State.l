;; Auto-generated. Do not edit!


(when (boundp 'Eurobot_msgs::State)
  (if (not (find-package "EUROBOT_MSGS"))
    (make-package "EUROBOT_MSGS"))
  (shadow 'State (find-package "EUROBOT_MSGS")))
(unless (find-package "EUROBOT_MSGS::STATE")
  (make-package "EUROBOT_MSGS::STATE"))

(in-package "ROS")
;;//! \htmlinclude State.msg.html


(defclass Eurobot_msgs::State
  :super ros::object
  :slots (_state ))

(defmethod Eurobot_msgs::State
  (:init
   (&key
    ((:state __state) "")
    )
   (send-super :init)
   (setq _state (string __state))
   self)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; string _state
    4 (length _state)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _state
       (write-long (length _state) s) (princ _state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get Eurobot_msgs::State :md5sum-) "af6d3a99f0fbeb66d3248fa4b3e675fb")
(setf (get Eurobot_msgs::State :datatype-) "Eurobot_msgs/State")
(setf (get Eurobot_msgs::State :definition-)
      "string state 
")



(provide :Eurobot_msgs/State "af6d3a99f0fbeb66d3248fa4b3e675fb")


