; Auto-generated. Do not edit!


(cl:in-package argos_bridge-msg)


;//! \htmlinclude LedsColor.msg.html

(cl:defclass <LedsColor> (roslisp-msg-protocol:ros-message)
  ((color
    :reader color
    :initarg :color
    :type std_msgs-msg:ColorRGBA
    :initform (cl:make-instance 'std_msgs-msg:ColorRGBA))
   (led_number
    :reader led_number
    :initarg :led_number
    :type cl:integer
    :initform 0)
   (intensity
    :reader intensity
    :initarg :intensity
    :type cl:fixnum
    :initform 0))
)

(cl:defclass LedsColor (<LedsColor>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LedsColor>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LedsColor)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name argos_bridge-msg:<LedsColor> is deprecated: use argos_bridge-msg:LedsColor instead.")))

(cl:ensure-generic-function 'color-val :lambda-list '(m))
(cl:defmethod color-val ((m <LedsColor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argos_bridge-msg:color-val is deprecated.  Use argos_bridge-msg:color instead.")
  (color m))

(cl:ensure-generic-function 'led_number-val :lambda-list '(m))
(cl:defmethod led_number-val ((m <LedsColor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argos_bridge-msg:led_number-val is deprecated.  Use argos_bridge-msg:led_number instead.")
  (led_number m))

(cl:ensure-generic-function 'intensity-val :lambda-list '(m))
(cl:defmethod intensity-val ((m <LedsColor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argos_bridge-msg:intensity-val is deprecated.  Use argos_bridge-msg:intensity instead.")
  (intensity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LedsColor>) ostream)
  "Serializes a message object of type '<LedsColor>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'color) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'led_number)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'led_number)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'led_number)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'led_number)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'intensity)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LedsColor>) istream)
  "Deserializes a message object of type '<LedsColor>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'color) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'led_number)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'led_number)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'led_number)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'led_number)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'intensity)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LedsColor>)))
  "Returns string type for a message object of type '<LedsColor>"
  "argos_bridge/LedsColor")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LedsColor)))
  "Returns string type for a message object of type 'LedsColor"
  "argos_bridge/LedsColor")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LedsColor>)))
  "Returns md5sum for a message object of type '<LedsColor>"
  "62998bbed77dd665705d07b81ccc0148")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LedsColor)))
  "Returns md5sum for a message object of type 'LedsColor"
  "62998bbed77dd665705d07b81ccc0148")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LedsColor>)))
  "Returns full string definition for message of type '<LedsColor>"
  (cl:format cl:nil "std_msgs/ColorRGBA color~%uint32 led_number~%uint8 intensity~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LedsColor)))
  "Returns full string definition for message of type 'LedsColor"
  (cl:format cl:nil "std_msgs/ColorRGBA color~%uint32 led_number~%uint8 intensity~%~%================================================================================~%MSG: std_msgs/ColorRGBA~%float32 r~%float32 g~%float32 b~%float32 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LedsColor>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'color))
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LedsColor>))
  "Converts a ROS message object to a list"
  (cl:list 'LedsColor
    (cl:cons ':color (color msg))
    (cl:cons ':led_number (led_number msg))
    (cl:cons ':intensity (intensity msg))
))
