/*
 * Copyright (C) 2014 Reach Technology, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/*!
 * @file g2h_dip.h
 *
 * @brief This file contains the 4 position DIP switch driver device interface 
 * and fops functions.
 */

/*
 * Returns the value of the 4-position dip switch
 */
#ifndef __G2H_DIP_H__
#define __G2H_DIP_H__

int dipswitch_get_value(void);

#endif
