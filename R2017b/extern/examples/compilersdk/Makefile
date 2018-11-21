#
#  Builds MATLAB Compiler SDK example files (Windows, Linux, Mac)
#  Copyright 1997-2016 The MathWorks, Inc.
#
# You can invoke this Makefile using 
#  gmake [-f Makefile] MATLABROOT=[directory where MATLAB or MATLAB Runtime is installed]
#
# If you do not want to specify the MATLABROOT at the gmake command line 
# every time, you can define it by uncommenting the line below
# and assigning the correct root of MATLAB (with no trailing slash) on
# the right hand side.
#
# MATLABROOT	:=
#

ifeq ($(MATLABROOT),)
	$(error MATLABROOT must be defined on the command line [e.g.: "MATLABROOT=c:\matlab"], as an environment variable, or in Makefile)
endif

ifeq ($(OS),Windows_NT)
	EXE=.exe
endif

MCC=$(MATLABROOT)/bin/mcc
MBUILD=$(MATLABROOT)/bin/mbuild

C_TARGETS=matrixdriver triangle catcherror

CPP_TARGETS=matrixdriverp trianglep catcherrorp

STAND_ALONE_TARGETS=$(C_TARGETS) $(CPP_TARGETS)

MAC_APP_TARGET_DIRS=$(addsuffix .app,$(STAND_ALONE_TARGETS))
# Use the following target to build and test all the executables
bnt_exes: build_all
	@echo "*********** testing matrixdriver ***********"
	./matrixdriver 
	@echo "*********** testing matrixdriverp ***********"
	./matrixdriverp
	@echo "*********** testing triangle ***********"
	./triangle 
	@echo "*********** testing trianglep ***********"
	./trianglep 
	@echo "*********** testing catcherror ***********"
	./catcherror 
	@echo "*********** testing catcherrorp ***********"
	./catcherrorp 
	

# Use this target to build all the targets without running them
build_all: $(STAND_ALONE_TARGETS)

# Use this target to build all the C++ targets without running them
only_cpp: $(CPP_TARGETS)

# Use this target to build the C targets without running them
only_c: $(C_TARGETS)


# The following are the definitions for each target individually.

# This target will create a C shared library and header file, 
# then compile an executable that uses them.
matrixdriver$(EXE):  matrixdriver.c addmatrix.m eigmatrix.m multiplymatrix.m
	$(MCC) -l -o libmatrix addmatrix.m eigmatrix.m multiplymatrix.m
	$(MBUILD) matrixdriver.c -L. -lmatrix
	@rm -f *.o*

# This target is like the previous except that it creates
# a C++ shared library and header file.
matrixdriverp$(EXE):  matrixdriver.cpp addmatrix.m eigmatrix.m multiplymatrix.m
	$(MCC) -B cpplib:libmatrixp addmatrix.m eigmatrix.m multiplymatrix.m
	$(MBUILD) matrixdriver.cpp -L. -lmatrixp -output matrixdriverp
	@rm -f *.o*

# This target will create a C shared library and header file, 
# then compile an executable that uses them.
triangle$(EXE):  triangle.c sierpinski.m
	$(MCC) -l -o libtriangle sierpinski.m
	$(MBUILD) triangle.c -L. -ltriangle
	@rm -f *.o*

# This target is like the previous except that it creates
# a C++ shared library and header file.
# The xcode subdirectory under this directory
# contains code that incorporates this C++ shared library to 
# demonstrate how to use XCode on the Mac.
trianglep$(EXE):  triangle.cpp sierpinski.m
	$(MCC) -B cpplib:libtrianglep sierpinski.m
	$(MBUILD) triangle.cpp -L. -ltrianglep -output trianglep
	@rm -f *.o*

# This target will create a C shared library and header file, 
# then compile an executable that uses them.
catcherror$(EXE):  catcherror.c realacos.m reveal.m
	$(MCC) -l -o libcatcherror realacos.m reveal.m
	$(MBUILD) catcherror.c -L. -lcatcherror
	@rm -f *.o*

# This target is like the previous except that it creates
# a C++ shared library and header file.
catcherrorp$(EXE):  catcherror.cpp realacos.m reveal.m
	$(MCC) -B cpplib:libcatcherrorp realacos.m reveal.m
	$(MBUILD) catcherror.cpp -L. -lcatcherrorp -output catcherrorp
	@rm -f *.o*

clean:
	rm -f $(STAND_ALONE_TARGETS) $(MAC_APP_TARGET_DIRS)

reallyclean:
	rm -rf $(STAND_ALONE_TARGETS)				\
		magicsquare.h magicsquare.c magicsquare_main.c	\
		libPkg* libMult* libtriangle* libmatrix*	\
		*.xml *.ctf *_mcr 	\
		*_main.c *.exports lib*                         \
		collect_one_external.h mccExcludedFiles.log \
		output $(MAC_APP_TARGET_DIRS)

.DEFAULT_GOAL := build_all