#**************************  M a k e f i l e ********************************
#  
#         Author: ub
#          $Date: 2004/11/29 11:34:21 $
#      $Revision: 1.1 $
#  
#    Description: makefile descriptor for chameleon Linux kernel module
#                      
#-----------------------------------------------------------------------------
#   Copyright (c) 2019, MEN Mikro Elektronik GmbH
#*****************************************************************************
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

MAK_NAME=lx_z135

MAK_LIBS=

MAK_SWITCH =

MAK_INCL=$(MEN_INC_DIR)/../../NATIVE/MEN/men_chameleon.h 

MAK_INP1=men_mdis_z135$(INP_SUFFIX)

MAK_INP=$(MAK_INP1)
