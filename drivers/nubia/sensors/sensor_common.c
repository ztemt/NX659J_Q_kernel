/*
* This file is part of the doubule sensor driver.
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
* 02110-1301 USA
*
*Reversion
*
====================================================================================
*/

#include "sensor_common.h"

int sensor_read_file(char *file_path, char *read_buf, int count)
{
	struct file *file_p;
	mm_segment_t old_fs;
	int vfs_retval = -EINVAL;

	if (NULL == file_path) {
		SENSOR_LOG_ERROR("file_path is NULL\n");
		return -EINVAL;
	}

	file_p = filp_open(file_path, O_RDONLY , 0444);
	if (IS_ERR(file_p)) {
		SENSOR_LOG_INFO("file does not exist\n");
		return -EINVAL;
	} 

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	file_p->f_pos = 0;
	vfs_retval = vfs_read(file_p, (char*)read_buf, count, &file_p->f_pos);
	if (vfs_retval < 0) {
		SENSOR_LOG_ERROR("[read file <%s>failed]\n",file_path);
		goto file_close;
	}

file_close:
	set_fs(old_fs);
	filp_close(file_p, NULL);

	return vfs_retval;
}

int sensor_write_file(char *file_path, const char *write_buf, int count)
{
	struct file *file_p;
	mm_segment_t old_fs;
	int vfs_retval = -EINVAL;

	if (NULL == file_path) {
		SENSOR_LOG_ERROR("file_path is NULL\n");
		return -EINVAL;
	}

	file_p = filp_open(file_path, O_CREAT|O_RDWR|O_TRUNC , 0666);
	if (IS_ERR(file_p)) {
		SENSOR_LOG_ERROR("[open file <%s>failed]\n",file_path);
		goto error;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_retval = vfs_write(file_p, (char*)write_buf, count, &file_p->f_pos);
	if (vfs_retval < 0) {
		SENSOR_LOG_ERROR("[write file <%s>failed]\n",file_path);
		goto file_close;
	}

file_close:
	set_fs(old_fs);
	filp_close(file_p, NULL);
error:
	return vfs_retval;
}

int sensor_create_sysfs_interfaces(struct device *dev,
	struct device_attribute *attr, int size)
{
	int i;
	for (i = 0; i < size; i++)
		if (device_create_file(dev, attr + i))
			goto exit;
	return 0;
exit:
	for (; i >= 0 ; i--)
		device_remove_file(dev, attr + i);
	SENSOR_LOG_ERROR("failed to create sysfs interface\n");
	return -ENODEV;
}

void sensor_remove_sysfs_interfaces(struct device *dev,
	struct device_attribute *attr, int size)
{
	int i;
	for (i = 0; i < size; i++)
		device_remove_file(dev, attr + i);
}


