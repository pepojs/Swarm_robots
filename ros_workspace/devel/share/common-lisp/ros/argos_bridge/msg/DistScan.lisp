; Auto-generated. Do not edit!


(cl:in-package argos_bridge-msg)


;//! \htmlinclude DistScan.msg.html

(cl:defclass <DistScan> (roslisp-msg-protocol:ros-message)
  ((range
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

(cl:defclass DistScan (<DistScan>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DistScan>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DistScan)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name argos_bridge-msg:<DistScan> is deprecated: use argos_bridge-msg:DistScan instead.")))

(cl:ensure-generic-function 'range-val :lambda-list '(m))
(cl:defmethod range-val ((m <DistScan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argos_bridge-msg:range-val is deprecated.  Use argos_bridge-msg:range instead.")
  (range m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <DistScan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argos_bridge-msg:angle-val is deprecated.  Use argos_bridge-msg:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DistScan>) ostream)
  "Serializes a message object of type '<DistScan>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DistScan>) istream)
  "Deserializes a message object of type '<DistScan>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DistScan>)))
  "Returns string type for a message object of type '<DistScan>"
  "argos_bridge/DistScan")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DistScan)))
  "Returns string type for a message object of type 'DistScan"
  "argos_bridge/DistScan")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DistScan>)))
  "Returns md5sum for a message object of type '<DistScan>"
  "5d665e328703e395c9fbb21d2decd1b5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DistScan)))
  "Returns md5sum for a message object of type 'DistScan"
  "5d665e328703e395c9fbb21d2decd1b5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DistScan>)))
  "Returns full string definition for message of type '<DistScan>"
  (cl:format cl:nil "float32 range~%float32 angle~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DistScan)))
  "Returns full string definition for message of type 'DistScan"
  (cl:format cl:nil "float32 range~%float32 angle~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DistScan>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DistScan>))
  "Converts a ROS message object to a list"
  (cl:list 'DistScan
    (cl:cons ':range (range msg))
    (cl:cons ':angle (angle msg))
))
