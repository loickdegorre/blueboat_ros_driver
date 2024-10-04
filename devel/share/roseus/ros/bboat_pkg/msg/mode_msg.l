;; Auto-generated. Do not edit!


(when (boundp 'bboat_pkg::mode_msg)
  (if (not (find-package "BBOAT_PKG"))
    (make-package "BBOAT_PKG"))
  (shadow 'mode_msg (find-package "BBOAT_PKG")))
(unless (find-package "BBOAT_PKG::MODE_MSG")
  (make-package "BBOAT_PKG::MODE_MSG"))

(in-package "ROS")
;;//! \htmlinclude mode_msg.msg.html


(defclass bboat_pkg::mode_msg
  :super ros::object
  :slots (_mode _mission ))

(defmethod bboat_pkg::mode_msg
  (:init
   (&key
    ((:mode __mode) "")
    ((:mission __mission) "")
    )
   (send-super :init)
   (setq _mode (string __mode))
   (setq _mission (string __mission))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:mission
   (&optional __mission)
   (if __mission (setq _mission __mission)) _mission)
  (:serialization-length
   ()
   (+
    ;; string _mode
    4 (length _mode)
    ;; string _mission
    4 (length _mission)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _mode
       (write-long (length _mode) s) (princ _mode s)
     ;; string _mission
       (write-long (length _mission) s) (princ _mission s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _mode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _mode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _mission
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _mission (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get bboat_pkg::mode_msg :md5sum-) "6609a085032a3aa05e4e4147b4c8d4d3")
(setf (get bboat_pkg::mode_msg :datatype-) "bboat_pkg/mode_msg")
(setf (get bboat_pkg::mode_msg :definition-)
      "string mode
string mission
")



(provide :bboat_pkg/mode_msg "6609a085032a3aa05e4e4147b4c8d4d3")


