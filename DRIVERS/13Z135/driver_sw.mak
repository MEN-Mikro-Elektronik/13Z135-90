#**************************  M a k e f i l e ********************************
#  
#         Author: ts
#          $Date: 2007/04/13 13:40:23 $
#      $Revision: 1.1 $
#  
#    Description: makefile descriptor for swapped 16Z025 Module
#                      
#---------------------------------[ History ]---------------------------------
#
#
#-----------------------------------------------------------------------------
#   (c) Copyright 2007 by MEN mikro elektronik GmbH, Nuremberg, Germany 
#*****************************************************************************

MAK_NAME=lx_z135_sw

MAK_LIBS=

MAK_SWITCH=$(SW_PREFIX)MAC_MEM_MAPPED \
	   $(SW_PREFIX)MAC_BYTESWAP

MAK_INCL=$(MEN_INC_DIR)/../../NATIVE/MEN/men_chameleon.h 

MAK_INP1=men_mdis_z135$(INP_SUFFIX)

MAK_INP=$(MAK_INP1)
