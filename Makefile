FILES = utility.py \
	transform.py \
	manipblty.py \
	Quaternion.py \
	jacobian.py \
	trajectory.py \
	kinematics.py \
	dynamics.py \
	wtrans.py \
	Link.py \
	Robot.py \
	plot.py \
	puma560.py \
	puma560akb.py \
	stanford.py \
	phantomx.py \
	twolink.py \
	fourlink2d.py \
	fourlink3d.py \
	parsedemo.py \
	testparser.py

PFILES = $(addprefix robot/, $(FILES))

VERSION = $(shell svn info -r HEAD | grep Revision | awk '{print $$2}')
PASSWORD = $(shell cat .password)

all:
	echo $(VERSION)

doc: 
	epydoc $(PFILES)
	zip -r rtbpy-doc-$(VERSION) html

