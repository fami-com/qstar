;;; generics.lisp 

;;; Copyright (C) 2008-2018  David O'Toole

;; Author: David O'Toole <dto@xelf.me>

;; Permission is hereby granted, free of charge, to any person obtaining a copy
;; of this software and associated documentation files (the "Software"), to deal
;; in the Software without restriction, including without limitation the rights
;; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
;; copies of the Software, and to permit persons to whom the Software is
;; furnished to do so, subject to the following conditions:

;; The above copyright notice and this permission notice shall be included in all
;; copies or substantial portions of the Software.

;; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
;; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
;; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
;; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
;; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
;; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
;; SOFTWARE.

(in-package :qstar)

(defgeneric location
  (NODE)
  (:documentation "Return as values the X,Y location of NODE."))

(defgeneric backward
  (NODE DISTANCE)
  (:documentation "Move NODE backward DISTANCE units along the aim direction."))

(defgeneric bounding-box
  (NODE)
  (:documentation "Return as multiple values the coordinates of the bounding box for
NODE. These are given in the order (TOP LEFT RIGHT BOTTOM)."))

(defgeneric center-point
  (NODE)
  (:documentation "Return as multiple values the coordinates of the NODE's center point."))

(defgeneric collide
  (NODE OTHER-NODE)
  (:documentation "This method is invoked when NODE collides with OTHER-NODE."))

(defgeneric colliding-with-p
  (NODE OTHER-NODE)
  (:documentation "Return non-nil if NODE's bounding box touches OTHER-NODE's bounding
box."))

(defgeneric colliding-with-rectangle-p
  (NODE TOP LEFT WIDTH HEIGHT)
  (:documentation "Return non-nil if NODE is colliding with the given rectangle."))

(defgeneric destroy
  (NODE)
  (:documentation "Destroy this NODE and remove it from any buffers."))

(defgeneric distance-between
  (NODE OTHER-NODE)
  (:documentation "Returns the distance between NODE and OTHER-NODE's center points."))

(defgeneric forward
  (NODE DISTANCE)
  (:documentation "Move the NODE forward along its current heading for DISTANCE units."))

(defgeneric handle-collision
  (NODE OTHER-NODE)
  (:documentation "Wraparound for collision handling. You shouldn't need to use this
explicitly."))

(defgeneric heading-between
  (NODE OTHER-NODE)
  (:documentation "Return the angle (in radians) of the ray from NODE to OTHER-NODE."))

(defgeneric move
  (NODE HEADING DISTANCE)
  (:documentation "Move the NODE toward HEADING by DISTANCE units."))

(defgeneric move-to
  (NODE X Y &OPTIONAL Z)
  (:documentation "Move the NODE to the point X,Y,Z."))

(defgeneric move-toward
  (NODE DIRECTION &OPTIONAL (STEPS))
  (:documentation "Move the node STEPS units in compass direction
DIRECTION."))

(defgeneric resize
  (NODE WIDTH HEIGHT)
  (:documentation "Resize the NODE to be WIDTH by HEIGHT units."))

(defgeneric turn-left
  (NODE RADIANS)
  (:documentation "Increase heading by RADIANS."))

(defgeneric turn-right
  (NODE RADIANS)
  (:documentation "Decrease heading by RADIANS."))

(defgeneric will-obstruct-p
  (NODE PATH-FINDER)
  (:documentation "Returns non-nil when the node NODE will obstruct
PATH-FINDER during path finding."))

(defgeneric aim
  (NODE HEADING)
  (:documentation "Set the NODE's current heading to HEADING."))

(defgeneric aim-at
  (NODE OTHER-NODE)
  (:documentation "Set the NODE's heading to aim at the OTHER-NODE."))

(defgeneric x
  (NODE)
  (:documentation "Return the current x-coordinate of the NODE."))

(defgeneric y
  (NODE)
  (:documentation "Return the current y-coordinate of the NODE."))

(defgeneric z
  (NODE)
  (:documentation "Return the current z-coordinate of the NODE."))
