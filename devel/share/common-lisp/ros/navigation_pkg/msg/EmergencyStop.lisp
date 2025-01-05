; Auto-generated. Do not edit!


(cl:in-package navigation_pkg-msg)


;//! \htmlinclude EmergencyStop.msg.html

(cl:defclass <EmergencyStop> (roslisp-msg-protocol:ros-message)
  ((signal
    :reader signal
    :initarg :signal
    :type cl:string
    :initform ""))
)

(cl:defclass EmergencyStop (<EmergencyStop>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EmergencyStop>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EmergencyStop)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navigation_pkg-msg:<EmergencyStop> is deprecated: use navigation_pkg-msg:EmergencyStop instead.")))

(cl:ensure-generic-function 'signal-val :lambda-list '(m))
(cl:defmethod signal-val ((m <EmergencyStop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation_pkg-msg:signal-val is deprecated.  Use navigation_pkg-msg:signal instead.")
  (signal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EmergencyStop>) ostream)
  "Serializes a message object of type '<EmergencyStop>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'signal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'signal))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EmergencyStop>) istream)
  "Deserializes a message object of type '<EmergencyStop>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'signal) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'signal) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EmergencyStop>)))
  "Returns string type for a message object of type '<EmergencyStop>"
  "navigation_pkg/EmergencyStop")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EmergencyStop)))
  "Returns string type for a message object of type 'EmergencyStop"
  "navigation_pkg/EmergencyStop")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EmergencyStop>)))
  "Returns md5sum for a message object of type '<EmergencyStop>"
  "9591b8ace81feee36c93130ad3fe6a19")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EmergencyStop)))
  "Returns md5sum for a message object of type 'EmergencyStop"
  "9591b8ace81feee36c93130ad3fe6a19")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EmergencyStop>)))
  "Returns full string definition for message of type '<EmergencyStop>"
  (cl:format cl:nil "string signal~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EmergencyStop)))
  "Returns full string definition for message of type 'EmergencyStop"
  (cl:format cl:nil "string signal~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EmergencyStop>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'signal))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EmergencyStop>))
  "Converts a ROS message object to a list"
  (cl:list 'EmergencyStop
    (cl:cons ':signal (signal msg))
))
