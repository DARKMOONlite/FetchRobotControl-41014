; Auto-generated. Do not edit!


(cl:in-package fetchit_challenge-msg)


;//! \htmlinclude SchunkMachineResult.msg.html

(cl:defclass <SchunkMachineResult> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass SchunkMachineResult (<SchunkMachineResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SchunkMachineResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SchunkMachineResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fetchit_challenge-msg:<SchunkMachineResult> is deprecated: use fetchit_challenge-msg:SchunkMachineResult instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SchunkMachineResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fetchit_challenge-msg:success-val is deprecated.  Use fetchit_challenge-msg:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SchunkMachineResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fetchit_challenge-msg:message-val is deprecated.  Use fetchit_challenge-msg:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SchunkMachineResult>) ostream)
  "Serializes a message object of type '<SchunkMachineResult>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SchunkMachineResult>) istream)
  "Deserializes a message object of type '<SchunkMachineResult>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SchunkMachineResult>)))
  "Returns string type for a message object of type '<SchunkMachineResult>"
  "fetchit_challenge/SchunkMachineResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SchunkMachineResult)))
  "Returns string type for a message object of type 'SchunkMachineResult"
  "fetchit_challenge/SchunkMachineResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SchunkMachineResult>)))
  "Returns md5sum for a message object of type '<SchunkMachineResult>"
  "937c9679a518e3a18d831e57125ea522")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SchunkMachineResult)))
  "Returns md5sum for a message object of type 'SchunkMachineResult"
  "937c9679a518e3a18d831e57125ea522")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SchunkMachineResult>)))
  "Returns full string definition for message of type '<SchunkMachineResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SchunkMachineResult)))
  "Returns full string definition for message of type 'SchunkMachineResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SchunkMachineResult>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SchunkMachineResult>))
  "Converts a ROS message object to a list"
  (cl:list 'SchunkMachineResult
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
