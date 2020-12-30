; Auto-generated. Do not edit!


(cl:in-package argos_bridge-msg)


;//! \htmlinclude BaseGround.msg.html

(cl:defclass <BaseGround> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0)
   (offset_x
    :reader offset_x
    :initarg :offset_x
    :type cl:float
    :initform 0.0)
   (offset_y
    :reader offset_y
    :initarg :offset_y
    :type cl:float
    :initform 0.0))
)

(cl:defclass BaseGround (<BaseGround>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BaseGround>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BaseGround)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name argos_bridge-msg:<BaseGround> is deprecated: use argos_bridge-msg:BaseGround instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <BaseGround>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argos_bridge-msg:value-val is deprecated.  Use argos_bridge-msg:value instead.")
  (value m))

(cl:ensure-generic-function 'offset_x-val :lambda-list '(m))
(cl:defmethod offset_x-val ((m <BaseGround>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argos_bridge-msg:offset_x-val is deprecated.  Use argos_bridge-msg:offset_x instead.")
  (offset_x m))

(cl:ensure-generic-function 'offset_y-val :lambda-list '(m))
(cl:defmethod offset_y-val ((m <BaseGround>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argos_bridge-msg:offset_y-val is deprecated.  Use argos_bridge-msg:offset_y instead.")
  (offset_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BaseGround>) ostream)
  "Serializes a message object of type '<BaseGround>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'offset_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'offset_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BaseGround>) istream)
  "Deserializes a message object of type '<BaseGround>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'offset_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'offset_y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BaseGround>)))
  "Returns string type for a message object of type '<BaseGround>"
  "argos_bridge/BaseGround")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BaseGround)))
  "Returns string type for a message object of type 'BaseGround"
  "argos_bridge/BaseGround")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BaseGround>)))
  "Returns md5sum for a message object of type '<BaseGround>"
  "df359e3890070432de3f37977284cdda")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BaseGround)))
  "Returns md5sum for a message object of type 'BaseGround"
  "df359e3890070432de3f37977284cdda")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BaseGround>)))
  "Returns full string definition for message of type '<BaseGround>"
  (cl:format cl:nil "float32 value~%float32 offset_x~%float32 offset_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BaseGround)))
  "Returns full string definition for message of type 'BaseGround"
  (cl:format cl:nil "float32 value~%float32 offset_x~%float32 offset_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BaseGround>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BaseGround>))
  "Converts a ROS message object to a list"
  (cl:list 'BaseGround
    (cl:cons ':value (value msg))
    (cl:cons ':offset_x (offset_x msg))
    (cl:cons ':offset_y (offset_y msg))
))
