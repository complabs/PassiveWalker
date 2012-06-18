###############################################################################
#
#  @file      Makefile
#  @brief     Makefile for Lab5: Passive Walker
#  @author    Mikica Kocic
#  @version   0.2
#  @date      2012-06-15
#  @copyright GNU Public License.
#

###############################################################################
# Configure bullet library location; Modify BULLET variable

BULLET    := ./bullet
BT_INC    := -I$(BULLET)/include/bullet
BT_LIB    := -L$(BULLET)/lib -lBulletDynamics -lBulletCollision -lLinearMath

###############################################################################
# If you have non-standard freeglut library, add -I<include> and -L<lib> to it.
# and define -DBT_USE_FREEGLUT=1. See the commented example bellow!

GLUT_INC  := 
GLUT_LIB  := -lglut -lGLU -lGL

# FREEGLUT  := ../local
# GLUT_INC  := -I$(FREEGLUT)/include -DBT_USE_FREEGLUT=1
# GLUT_LIB  := -L$(FREEGLUT)/lib -lglut -lGLU -lGL

###############################################################################
# For verbose compilation (to see exectued commands) call make as: make Q=

Q := @

###############################################################################

CXXFLAGS  := -Isrc/OpenGL $(BT_INC) $(GLUT_INC)
LDFLAGS   := $(BT_LIB) $(GLUT_LIB)

SRC_DIR   := src
OBJ_DIR   := obj
BIN_DIR   := bin

###############################################################################

EXE_NAME := PassiveWalker

SRC_FILES := \
    Constants.cpp Machine.cpp PassiveWalker.cpp PassiveWalkerTestBed.cpp \
    PassiveWalkerApplication.cpp RagDoll.cpp Main.cpp \
    OpenGL/DemoApplication.cpp OpenGL/GLShapeDrawer.cpp \
    OpenGL/GLDebugDrawer.cpp OpenGL/GLDebugFont.cpp

###############################################################################

SRC := $(addprefix $(SRC_DIR)/,$(SRC_FILES))
OBJ := $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRC))

vpath %.cpp $(SRC_DIR)

#CXXFLAGS  := $(CXXFLAGS) $(addprefix -I,$(SRC_DIR))

.PHONY: all rebuild clean distclean tarball

###############################################################################

all : $(BIN_DIR)/$(EXE_NAME)

$(BIN_DIR)/$(EXE_NAME) : $(OBJ)
	@$(if $(Q), echo " [LD   ] " $@ )
	$(Q)g++ $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

clean :
	@$(if $(Q), echo " [RM   ] " $(OBJ_DIR)/\*.o )
	$(Q)rm -rf $(OBJ_DIR)/*.o
	@$(if $(Q), echo " [RM   ] " $(OBJ_DIR)/OpenGL/\*.o )
	$(Q)rm -rf $(OBJ_DIR)/OpenGL/*.o

distclean : clean
	@$(if $(Q), echo " [RM   ] " $(BIN_DIR)/$(EXE_NAME) )
	$(Q)rm -rf $(BIN_DIR)/$(EXE_NAME)

rebuild : clean all

tarball : distclean
	tar cvfz PassiveWalker-`date +%Y%m%d-%H%M%S`.tgz Makefile src bin obj

###############################################################################
# Dependencies (manual :)

Constants.o: Constants.cpp \
    Constants.h

Machine.o: Machine.cpp \
    Constants.h Machine.h

PassiveWalker.o: PassiveWalker.cpp \
    Constants.h Machine.h PassiveWalker.h

RagDoll.o: RagDoll.cpp \
    Constants.h Machine.h RagDoll.h

PassiveWalkerTestBed.o: PassiveWalkerTestBed.cpp \
    Constants.h Machine.h PassiveWalker.h \
    PassiveWalkerTestBed.h

PassiveWalkerApplication.o: PassiveWalkerApplication.cpp \
    Constants.h Machine.h PassiveWalker.h \
    PassiveWalkerTestBed.h PassiveWalkerApplication.h \
    GLUT_Framework.h OpenGL/GlutStuff.h OpenGL/DemoApplication.h

Main.o: Main.cpp \
    Constants.h Machine.h PassiveWalker.h \
    PassiveWalkerTestBed.h PassiveWalkerApplication.h \
    GLUT_Framework.h OpenGL/GlutStuff.h

###############################################################################

OpenGL/DemoApplication.o: OpenGL/DemoApplication.cpp \
    OpenGL/GlutStuff.h OpenGL/DemoApplication.h

OpenGL/GLDebugDrawer.o: OpenGL/GLDebugDrawer.cpp \
    OpenGL/GlutStuff.h OpenGL/GLDebugDrawer.h

OpenGL/GLDebugFont.o: OpenGL/GLDebugFont.cpp \
    OpenGL/GlutStuff.h OpenGL/GLDebugFont.h

OpenGL/GLShapeDrawer.o: OpenGL/GLShapeDrawer.cpp \
    OpenGL/GlutStuff.h OpenGL/GLShapeDrawer.h

###############################################################################
# Make goal definitions for each directory in OBJ_DIR

define make-goal
$1/%.o: %.cpp
	@$(if $(Q), echo " [C++  ] " $$@ )
	$(Q)g++ $(CXXFLAGS) -c $$< -o $$@
endef

$(foreach bdir,$(OBJ_DIR),$(eval $(call make-goal,$(bdir))))

###############################################################################
