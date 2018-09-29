;;; -*- Mode: Lisp; -*-

(defpackage :qstar-asd)

(in-package :qstar-asd)

(asdf:defsystem qstar
  :name "qstar"
  :version "4.1"
  :maintainer "David T O'Toole <dto@qstar.me>"
  :author "David T O'Toole <dto@qstar.me>"
  :license "MIT"
  :description "quadtree-based 2-D collision detection and A-star pathfinding"
  :serial t
  :components ((:file "package")
	       (:file "generics" :depends-on ("package"))
	       (:file "quadtree" :depends-on ("generics"))
	       (:file "astar" :depends-on ("package" "quadtree"))))
 

