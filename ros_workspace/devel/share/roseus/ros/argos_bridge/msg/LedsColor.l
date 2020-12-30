;; Auto-generated. Do not edit!


(when (boundp 'argos_bridge::LedsColor)
  (if (not (find-package "ARGOS_BRIDGE"))
    (make-package "ARGOS_BRIDGE"))
  (shadow 'LedsColor (find-package "ARGOS_BRIDGE")))
(unless (find-package "ARGOS_BRIDGE::LEDSCOLOR")
  (make-package "ARGOS_BRIDGE::LEDSCOLOR"))

(in-package "ROS")
;;//! \htmlinclude LedsColor.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass argos_bridge::LedsColor
  :super ros::object
  :slots (_color _led_number _intensity ))

(defmethod argos_bridge::LedsColor
  (:init
   (&key
    ((:color __color) (instance std_msgs::ColorRGBA :init))
    ((:led_number __led_number) 0)
    ((:intensity __intensity) 0)
    )
   (send-super :init)
   (setq _color __color)
   (setq _led_number (round __led_number))
   (setq _intensity (round __intensity))
   self)
  (:color
   (&rest __color)
   (if (keywordp (car __color))
       (send* _color __color)
     (progn
       (if __color (setq _color (car __color)))
       _color)))
  (:led_number
   (&optional __led_number)
   (if __led_number (setq _led_number __led_number)) _led_number)
  (:intensity
   (&optional __intensity)
   (if __intensity (setq _intensity __intensity)) _intensity)
  (:serialization-length
   ()
   (+
    ;; std_msgs/ColorRGBA _color
    (send _color :serialization-length)
    ;; uint32 _led_number
    4
    ;; uint8 _intensity
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/ColorRGBA _color
       (send _color :serialize s)
     ;; uint32 _led_number
       (write-long _led_number s)
     ;; uint8 _intensity
       (write-byte _intensity s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/ColorRGBA _color
     (send _color :deserialize buf ptr-) (incf ptr- (send _color :serialization-length))
   ;; uint32 _led_number
     (setq _led_number (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _intensity
     (setq _intensity (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get argos_bridge::LedsColor :md5sum-) "62998bbed77dd665705d07b81ccc0148")
(setf (get argos_bridge::LedsColor :datatype-) "argos_bridge/LedsColor")
(setf (get argos_bridge::LedsColor :definition-)
      "std_msgs/ColorRGBA color
uint32 led_number
uint8 intensity

================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a

")



(provide :argos_bridge/LedsColor "62998bbed77dd665705d07b81ccc0148")


