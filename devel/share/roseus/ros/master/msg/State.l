;; Auto-generated. Do not edit!


(when (boundp 'master::State)
  (if (not (find-package "MASTER"))
    (make-package "MASTER"))
  (shadow 'State (find-package "MASTER")))
(unless (find-package "MASTER::STATE")
  (make-package "MASTER::STATE"))

(in-package "ROS")
;;//! \htmlinclude State.msg.html


(defclass master::State
  :super ros::object
  :slots (_state ))

(defmethod master::State
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

(setf (get master::State :md5sum-) "af6d3a99f0fbeb66d3248fa4b3e675fb")
(setf (get master::State :datatype-) "master/State")
(setf (get master::State :definition-)
      "string state 
")



(provide :master/State "af6d3a99f0fbeb66d3248fa4b3e675fb")


