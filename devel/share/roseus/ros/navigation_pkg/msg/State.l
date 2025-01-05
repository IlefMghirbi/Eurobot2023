;; Auto-generated. Do not edit!


(when (boundp 'navigation_pkg::State)
  (if (not (find-package "NAVIGATION_PKG"))
    (make-package "NAVIGATION_PKG"))
  (shadow 'State (find-package "NAVIGATION_PKG")))
(unless (find-package "NAVIGATION_PKG::STATE")
  (make-package "NAVIGATION_PKG::STATE"))

(in-package "ROS")
;;//! \htmlinclude State.msg.html


(defclass navigation_pkg::State
  :super ros::object
  :slots (_state ))

(defmethod navigation_pkg::State
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

(setf (get navigation_pkg::State :md5sum-) "af6d3a99f0fbeb66d3248fa4b3e675fb")
(setf (get navigation_pkg::State :datatype-) "navigation_pkg/State")
(setf (get navigation_pkg::State :definition-)
      "string state 
")



(provide :navigation_pkg/State "af6d3a99f0fbeb66d3248fa4b3e675fb")


