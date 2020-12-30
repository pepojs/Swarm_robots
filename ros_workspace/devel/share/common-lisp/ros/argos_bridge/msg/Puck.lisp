; Auto-generated. Do not edit!


(cl:in-package argos_bridge-msg)


;//! \htmlinclude Puck.msg.html

(cl:defclass <Puck> (roslisp-msg-protocol:ros-message)
  ((color
    :reader color
    :initarg :color
    :type std_msgs-msg:ColorRGBA
    :initform (cl:make-instance 'std_msgs-msg:ColorRGBA))
   (range
    :reader range
    :initarg :range
    :type cl:float
    :initform 0.0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass Puck (<Puck>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Puck>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Puck)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name argos_bridge-msg:<Puck> is deprecated: use argos_bridge-msg:Puck instead.")))

(cl:ensure-generic-function 'color-val :lambda-list '(m))
(cl:defmethod color-val ((m <Puck>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argos_bridge-msg:color-val is deprecated.  Use argos_bridge-msg:color instead.")
  (color m))

(cl:ensure-generic-function 'range-val :lambda-list '(m))
(cl:defmethod range-val ((m <Puck>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argos_bridge-msg:range-val is deprecated.  Use argos_bridge-msg:range instead.")
  (range m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <Puck>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argos_bridge-msg:angle-val is deprecated.  Use argos_bridge-msg:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Puck>) ostream)
  "Serializes a message object of type '<Puck>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'color) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Puck>) istream)
  "Deserializes a message object of type '<Puck>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'color) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'range) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Puck>)))
  "Returns string type for a message object of type '<Puck>"
  "argos_bridge/Puck")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Puck)))
  "Returns string type for a message object of type 'Puck"
  "argos_bridge/Puck")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Puck>)))
  "Returns md5sum for a message object of type '<Puck>"
  "4ed42de6679ef0abec7e884aca1135c6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Puck)))
  "Returns md5sum for a message object of type 'Puck"
  "4ed42de6679ef0abec7e884aca1135c6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Puck>)))
  "Returns full string definition for message of type '<Puck>"
  (cl:format cl:nil "std_msgs/ColorRGBA color~%float32 range~%float32 angle~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Puck)))
  "Returns full string definition for message of type 'Puck"
  (cl:format cl:nil "std_msgs/ColorRGBA color~%float32 range~%float32 angle~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Puck>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'color))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Puck>))
  "Converts a ROS message object to a list"
  (cl:list 'Puck
    (cl:cons ':color (color msg))
    (cl:cons ':range (range msg))
    (cl:cons ':angle (angle msg))
))
