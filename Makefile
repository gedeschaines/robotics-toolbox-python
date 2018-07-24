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

VERSION = $(shell ./getvers.sh)

vers:
	@echo "RTB for Python version in setup.py is $(VERSION)"

all:
	@echo "RTB for Python version in setup.py is $(VERSION)"

# Note: Using epydoc to produce code documentation is only
#       advisable after refinements have been made to import
#       statements to eleminate inclusion of documentation
#       for non-robot module code.

doc: 
	epydoc $(PFILES)
	zip -r rtb4python-doc-$(VERSION) html
