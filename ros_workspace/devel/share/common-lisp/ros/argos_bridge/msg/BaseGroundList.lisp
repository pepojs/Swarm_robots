; Auto-generated. Do not edit!


(cl:in-package argos_bridge-msg)


;//! \htmlinclude BaseGroundList.msg.html

(cl:defclass <BaseGroundList> (roslisp-msg-protocol:ros-message)
  ((n
    :reader n
    :initarg :n
    :type cl:integer
    :initform 0)
   (baseGrounds
    :reader baseGrounds
    :initarg :baseGrounds
    :type (cl:vector argos_bridge-msg:BaseGround)
   :initform (cl:make-array 0 :element-type 'argos_bridge-msg:BaseGround :initial-element (cl:make-instance 'argos_bridge-msg:BaseGround))))
)

(cl:defclass BaseGroundList (<BaseGroundList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BaseGroundList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BaseGroundList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name argos_bridge-msg:<BaseGroundList> is deprecated: use argos_bridge-msg:BaseGroundList instead.")))

(cl:ensure-generic-function 'n-val :lambda-list '(m))
(cl:defmethod n-val ((m <BaseGroundList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argos_bridge-msg:n-val is deprecated.  Use argos_bridge-msg:n instead.")
  (n m))

(cl:ensure-generic-function 'baseGrounds-val :lambda-list '(m))
(cl:defmethod baseGrounds-val ((m <BaseGroundList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argos_bridge-msg:baseGrounds-val is deprecated.  Use argos_bridge-msg:baseGrounds instead.")
  (baseGrounds m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BaseGroundList>) ostream)
  "Serializes a message object of type '<BaseGroundList>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'n)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'n)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'n)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'baseGrounds))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'baseGrounds))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BaseGroundList>) istream)
  "Deserializes a message object of type '<BaseGroundList>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'n)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'n)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'n)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'baseGrounds) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'baseGrounds)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'argos_bridge-msg:BaseGround))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BaseGroundList>)))
  "Returns string type for a message object of type '<BaseGroundList>"
  "argos_bridge/BaseGroundList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BaseGroundList)))
  "Returns string type for a message object of type 'BaseGroundList"
  "argos_bridge/BaseGroundList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BaseGroundList>)))
  "Returns md5sum for a message object of type '<BaseGroundList>"
  "69c1c089ae74b076579a8bd75e220110")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BaseGroundList)))
  "Returns md5sum for a message object of type 'BaseGroundList"
  "69c1c089ae74b076579a8bd75e220110")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BaseGroundList>)))
  "Returns full string definition for message of type '<BaseGroundList>"
  (cl:format cl:nil "uint32 n~%BaseGround[] baseGrounds~%~%================================================================================~%MSG: argos_bridge/BaseGround~%float32 value~%float32 offset_x~%float32 offset_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BaseGroundList)))
  "Returns full string definition for message of type 'BaseGroundList"
  (cl:format cl:nil "uint32 n~%BaseGround[] baseGrounds~%~%================================================================================~%MSG: argos_bridge/BaseGround~%float32 value~%float32 offset_x~%float32 offset_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BaseGroundList>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'baseGrounds) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BaseGroundList>))
  "Converts a ROS message object to a list"
  (cl:list 'BaseGroundList
    (cl:cons ':n (n msg))
    (cl:cons ':baseGrounds (baseGrounds msg))
))
