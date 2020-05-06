/*
 * fs/fuse/file_observer.c
 *
 * Copyright (c) 2017 Nubia Technology Co. Ltd
 *   Author: wuzhibin
 *
 * This file is dual licensed.  It may be redistributed and/or modified
 * under the terms of the Apache 2.0 License OR version 2 of the GNU
 * General Public License.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/parser.h>

#include <linux/netlink.h>
#include <net/sock.h>
#include "file_observer.h"
#include <linux/mm.h>
#include <linux/list.h>
#include <linux/mount.h>
#include <../mount.h>
#include <linux/mtd/mtd.h>
#include "fuse_i.h"
#include "linux/jiffies.h"

static struct sock *netlink_fd;
static int g_pid = -1;
static bool gEnable = true;

static Last_Msg g_last_msg;

#define DEBUG 0

extern unsigned long volatile jiffies;


#define EXTERNAL_STORAGE_PREFIX "/storage"

static void fuse_netlink_process_packet(struct nlmsghdr *nl){
    switch(nl->nlmsg_type)	{
            case NL_TYPE_SET_DEST_ADDR:
                printk(KERN_DEBUG"I got the FileObserverD pid %d\r\n", nl->nlmsg_pid);
                g_pid = nl->nlmsg_pid;
                break;
            case NL_TYPE_SET_ENABLE:
                printk(KERN_DEBUG"Enable file observer pid: %d\r\n", nl->nlmsg_pid);
                gEnable = true;
                g_pid = nl->nlmsg_pid;
                break;
            case NL_TYPE_SET_DISABLE:
                printk(KERN_DEBUG"Disable file observer\r\n");
                gEnable = false;
                break;
            default:break;
    }
}
static void fuse_netlink_recv_packet(struct sk_buff *__skb){
    struct sk_buff *skb;
    struct nlmsghdr *nlhdr;
    skb = skb_get(__skb);
    if(skb->len >= sizeof(struct nlmsghdr))	{
        nlhdr = (struct nlmsghdr *)skb->data;
        if(nlhdr->nlmsg_len >= sizeof(struct nlmsghdr) &&	 __skb->len >= nlhdr->nlmsg_len)	{
            fuse_netlink_process_packet(nlhdr);
        }
    }	else
        printk(KERN_ALERT "Kernel receive msg length error!\n");
}
static struct netlink_kernel_cfg fo_netlink_cfg = {
        .input = fuse_netlink_recv_packet
};

int fuse_init_file_observer(void) {
    if(!gEnable)
        return 0;

    netlink_fd = netlink_kernel_create(&init_net, USER_NETLINK_CMD, &fo_netlink_cfg);
    if(NULL == netlink_fd)	{
        printk(KERN_ALERT "Init netlink!\n");
        return -1;
    }
    g_last_msg.data_len = 0;
    g_last_msg.ino = 0;
    g_last_msg.time = 0;
    g_last_msg.is_valid = false;
    return 0;
}
int fuse_exit_file_observer(void) {
    if(!gEnable)
        return 0;

    if(NULL != netlink_fd)	{
        netlink_kernel_release(netlink_fd);
    }
    netlink_fd = NULL;
    return 0;
}

static int fuse_nl_send_msg(const u8 *data, int len)    {
    struct nlmsghdr *rep;
    u8 *res;
    struct sk_buff *skb;

    if(g_pid == -1)
        return -1;
    if(data == NULL)
        return -1;
    if(len <=0)
        return -1;

    skb = nlmsg_new(len, GFP_KERNEL);
    if(!skb) {
        printk("nlmsg_new failed!!!\n");
        return -1;
    }

    #if DEBUG
    printk("Send Msg To User len:%d\n", len);
    #endif
    rep = nlmsg_put(skb, g_pid, 0, NLMSG_NOOP, len, 0);
    res = nlmsg_data(rep);
    memcpy(res, data, len);
    netlink_unicast(netlink_fd, skb, g_pid, MSG_DONTWAIT);
    return 0;
}

static ssize_t get_node_path_locked(struct dentry *dentry, char* buf, size_t bufsize) {
    const char* name;
    size_t namelen;
    ssize_t pathlen = 0;
    if(dentry == NULL) {
        return -1;
    }
    //printk("get_node_path_locked 22222 dentry->d_name.name: %s   d_iname:%s\r\n", dentry->d_name.name, dentry->d_iname);
    name = dentry->d_name.name;//d_iname;//
    namelen = strlen(name);
    if (bufsize < namelen + 1) {
        return -1;
    }
    //printk("--->%s\n", dentry->d_iname);
    if(dentry == dentry->d_parent) {
        return 0;
    } else {
        pathlen = get_node_path_locked(dentry->d_parent, buf, bufsize - namelen - 1);
        if (pathlen < 0) {
            return -1;
        }
        buf[pathlen++] = '/';
    }

    memcpy(buf + pathlen, name, namelen + 1); /* include trailing \0 */
    return pathlen + namelen;
}

static const char* fuse_getMountPoint(struct dentry *dentry) {
    struct super_block *sb = dentry->d_sb;
    struct mount *mnt;
    mnt = list_first_entry(&sb->s_mounts, typeof(*mnt), mnt_instance);
    if(mnt != NULL && mnt->mnt_mountpoint != NULL) {
         //printk("-------------->mount point:%s\n", mnt->mnt_mountpoint->d_name.name);
         return mnt->mnt_mountpoint->d_name.name;
    }
    return NULL;

}

static size_t fuse_getDentryFullPath(struct dentry *dentry, char* buf, size_t buffSize) {
    const char* mount_point = "";
    size_t mount_point_len = 0;

    memset(buf, 0, buffSize);
    mount_point = fuse_getMountPoint(dentry);
    mount_point_len = strlen(EXTERNAL_STORAGE_PREFIX)+strlen(mount_point)+2;
    snprintf(buf, mount_point_len, "%s/%s", EXTERNAL_STORAGE_PREFIX, mount_point);
    if(get_node_path_locked(dentry, buf+mount_point_len-1, buffSize-mount_point_len-1) <= 0) {
        return 0;
    }
    //printk("sub path:%s\n", buf+mount_point_len-1);
    return strlen(buf);
}

static size_t  fuse_createMsg(FO_OP op, struct inode *inode, struct dentry *dentry,
                                                         struct fuse_file_creator* creator, char* buf, size_t buffSize,
                                                         struct dentry *old_dentry, struct dentry *new_dentry) {
    int len = 0;
    char*  offset = NULL;
    int fmCnt = 0;
    FO_Header* header  = NULL;
    FO_FragMent *fm_ino = NULL, *fm_size =NULL, *fm_uid=NULL,
                         *fm_pid=NULL, *fm_tname=NULL, *fm_pname=NULL,
                         *fm_cuid=NULL, *fm_cpid=NULL, *fm_path = NULL,
                         *fm_oldpath=NULL, *fm_newpath=NULL;
    struct task_struct *task = current;

    if(inode != NULL) {
        memset(buf, 0, buffSize);
        offset = buf;
        header = (FO_Header*)offset;
        header->flag[0] = 'F';
        header->flag[1] = 'O';
        header->flag[2] = 'M';
        header->flag[3] = 'G';
        header->op = op;

        offset += sizeof(FO_Header);
        #if DEBUG
        printk("buf:%p  offset:%p, header:%p\n", buf, offset, header);
        #endif


        fm_ino = (FO_FragMent*)offset;
        fm_ino->fm = FRAGEMENT_INO;
        offset += sizeof(FO_FragMent);
        fmCnt++;

        fm_size = (FO_FragMent*)offset;
        fm_size->fm = FRAGEMENT_SIZE;
        offset += sizeof(FO_FragMent);
        fmCnt++;

        fm_uid = (FO_FragMent*)offset;
        fm_uid->fm = FRAGEMENT_UID;
        offset += sizeof(FO_FragMent);
        fmCnt++;

        fm_pid = (FO_FragMent*)offset;
        fm_pid->fm = FRAGEMENT_PID;
        offset += sizeof(FO_FragMent);
        fmCnt++;

        if(creator != NULL) {
            fm_cuid = (FO_FragMent*)offset;
            fm_cuid->fm = FRAGEMENT_CUID;
            offset += sizeof(FO_FragMent);
            fmCnt++;

            fm_cpid = (FO_FragMent*)offset;
            fm_cpid->fm = FRAGEMENT_CPID;
            offset += sizeof(FO_FragMent);
            fmCnt++;
        }

        fm_tname = (FO_FragMent*)offset;
        fm_tname->fm = FRAGEMENT_TNAME;
        offset += sizeof(FO_FragMent);
        fmCnt++;

        fm_pname = (FO_FragMent*)offset;
        fm_pname->fm = FRAGEMENT_PNAME;
        offset += sizeof(FO_FragMent);
        fmCnt++;

        if(dentry != NULL) {
            fm_path = (FO_FragMent*)offset;
            fm_path->fm = FRAGEMENT_PATH;
            offset += sizeof(FO_FragMent);
            fmCnt++;
        }

        if(old_dentry != NULL) {
            fm_oldpath = (FO_FragMent*)offset;
            fm_oldpath->fm = FRAGEMENT_OLDPATH;
            offset += sizeof(FO_FragMent);
            fmCnt++;
        }

        if(new_dentry != NULL) {
            fm_newpath = (FO_FragMent*)offset;
            fm_newpath->fm = FRAGEMENT_NEWPATH;
            offset += sizeof(FO_FragMent);
            fmCnt++;
        }

        if(fm_ino != NULL) {
            memcpy((void*)offset, &(inode->i_ino), sizeof( unsigned long));
            fm_ino->offset = offset - buf;
            fm_ino->len = sizeof(unsigned long);
            offset += fm_ino->len;
        }
        if(fm_size != NULL) {
            memcpy((void*)offset, &(inode->i_size), sizeof(loff_t));
            fm_size->offset = offset - buf;
            fm_size->len = sizeof(loff_t);
            offset += fm_size->len;
        }
        if(fm_uid != NULL) {
            memcpy((void*)offset, &(current->cred->uid.val), sizeof(kuid_t));
            fm_uid->offset = offset - buf;
            fm_uid->len = sizeof(kuid_t);
            offset += fm_uid->len;
        }
        if(fm_pid != NULL) {
            memcpy((void*)offset, &(current->tgid), sizeof(pid_t));
            fm_pid->offset = offset - buf;
            fm_pid->len = sizeof(pid_t);
            offset += fm_pid->len;
        }

        if(creator != NULL && fm_cuid != NULL && fm_cpid != NULL) {
            memcpy((void*)offset, &(creator->uid), sizeof(uid_t));
            fm_cuid->offset = offset - buf;
            fm_cuid->len = sizeof(uid_t);
            offset += fm_cuid->len;

            memcpy((void*)offset, &(creator->pid), sizeof(pid_t));
            fm_cpid->offset = offset - buf;
            fm_cpid->len = sizeof(pid_t);
            offset += fm_cpid->len;
        }

        if(current->parent != NULL) {
            if((current->comm[0] == 'c' && current->comm[1] == 'p' && current->comm[2] == 0) ||
                (current->comm[0] == 'm' && current->comm[1] == 'v' && current->comm[2] == 0)) {
                task = current->parent;
            } else {
                task = current;
            }
        }

        if(fm_tname != NULL) {
            len = strlen(task->comm);
            memcpy((void*)offset, task->comm, len);
            fm_tname->offset = offset - buf;
            fm_tname->len = len;
            offset += fm_tname->len;
            offset++;
        }
        if(fm_pname != NULL) {
            len = get_cmdline(task, (void*)offset, buffSize-(offset-buf));
            fm_pname->offset = offset - buf;
            fm_pname->len = len;
            offset += fm_pname->len;
            offset++;
        }

        if(dentry != NULL && fm_path != NULL) {
            len = fuse_getDentryFullPath(dentry, (void*)offset, buffSize-(offset-buf));
            fm_path->offset = offset - buf;
            fm_path->len = len;
            offset += fm_path->len;
            offset++;
        }

        if(old_dentry != NULL && fm_oldpath != NULL) {
            len = fuse_getDentryFullPath(old_dentry, (void*)offset, buffSize-(offset-buf));
            fm_oldpath->offset = offset - buf;
            fm_oldpath->len = len;
            offset += fm_oldpath->len;
            offset++;
        }

        if(new_dentry != NULL && fm_newpath != NULL) {
            len = fuse_getDentryFullPath(new_dentry, (void*)offset, buffSize-(offset-buf));
            fm_newpath->offset = offset - buf;
            fm_newpath->len = len;
            offset += fm_newpath->len;
        }


        header->fmCnt = fmCnt;
        #if DEBUG
        if(fm_path != NULL) {
            printk("op:%d fmCnt:%d  ino:%lu pname:%s path:%s", header->op, header->fmCnt, *(unsigned long*)(buf+fm_ino->offset),  buf+fm_pname->offset, buf+fm_path->offset);
        } else if(fm_oldpath != NULL && fm_newpath != NULL) {
            printk("op:%d fmCnt:%d  ino:%lu pname:%s oldpath:%s  newpath:%s", header->op, header->fmCnt, *(unsigned long*)(buf+fm_ino->offset),  buf+fm_pname->offset, buf+fm_oldpath->offset, buf+fm_newpath->offset);
        }
        #endif

        return offset-buf;
    }
    return 0;
}


int fuse_post_file_create(struct dentry *dentry)  {
    char* data;
    size_t len = 0;

    if(!gEnable)
        return 0;

    if(g_pid == -1)
        return -1;

    if(dentry == NULL ||  dentry->d_inode == NULL) {
        return -1;
    }

    if(!S_ISREG(dentry->d_inode->i_mode)) {
        return -1;
    }
    #if DEBUG
    printk("FUSE thread name:%s\n", current->comm);
    #endif

    data = (char*)kmalloc(NL_MSG_MAX, GFP_KERNEL);
    if(data != NULL) {
        len = fuse_createMsg(OP_CREATE, dentry->d_inode, dentry, NULL, data, NL_MSG_MAX, NULL, NULL);
        fuse_nl_send_msg(data, len);
        kfree(data);
    }

    return 0;

}

#if 0
int fuse_post_file_open(struct file *file)  {

    struct fuse_inode_info* inode_info;
    struct fuse_file *file_info = file->private_data;

    if(!gEnable)
        return 0;

    if(g_pid == -1)
        return -1;
    if(file == NULL || file->f_path.dentry == NULL ||
        file->f_path.dentry->d_inode == NULL) {
        return -1;
    }

    if(!S_ISREG(file->f_path.dentry->d_inode->i_mode)) {
        return -1;
    }

    if(file_info == NULL) {
        return -1;
    }
    inode_info = (struct fuse_inode_info*)file->f_path.dentry->d_inode->i_private;//SDCARDFS_I(file->f_path.dentry->d_inode);
    printk("-222 open--->%p\n", inode_info);
    if(inode_info == NULL) {
        return -1;
    }
    if(inode_info->mask|FUSE_FS_CREATE) {
        printk("post_file_open  %lu  \n", file->f_path.dentry->d_inode->i_ino);
        file_info->mask|= FUSE_FS_CREATE;
        inode_info->mask &= ~FUSE_FS_CREATE;
        if(inode_info->mask|FUSE_RELEASE_PRIVATE) {
            printk("release private data\n");
            kfree(inode_info);
            file->f_path.dentry->d_inode->i_private = NULL;
        }
    }
    return 0;

}
#endif

int fuse_post_file_write(struct file *file)  {
    struct fuse_file* info;

    if(!gEnable)
        return 0;

    if(g_pid == -1)
        return -1;
    if(file == NULL || file->f_path.dentry == NULL ||
        file->f_path.dentry->d_inode == NULL) {
        return -1;
    }

    if(!S_ISREG(file->f_path.dentry->d_inode->i_mode)) {
        return -1;
    }

    //printk("post file write  is new:%lu\n", (file->f_path.dentry->d_inode->i_state & I_NEW));
    //show_stack(NULL, NULL);
    info = (struct fuse_file*)file->private_data;
    info->mask |= FUSE_FS_MODIFY;

    return 0;
}
int fuse_post_file_mkdir(struct inode *dir, struct dentry *dentry) {
    char* data;
    size_t len = 0;

    if(!gEnable)
        return 0;

    if(g_pid == -1)
        return -1;

    #if DEBUG
    printk("====mkdir %s\n", dentry->d_name.name);
    #endif

     if(dentry != NULL && dir != NULL) {
        data = (char*)kmalloc(NL_MSG_MAX, GFP_KERNEL);
        if(data != NULL) {
            len = fuse_createMsg(OP_MKDIR, dir, dentry, NULL, data, NL_MSG_MAX, NULL, NULL);
            fuse_nl_send_msg(data, len);
            kfree(data);
        }
    }
    return 0;
}
int fuse_post_file_rmdir(struct inode *dir, struct dentry *dentry) {
    char* data;
    size_t len = 0;

    if(!gEnable)
        return 0;

    if(g_pid == -1)
        return -1;

    #if DEBUG
    printk("====rmdir %s\n", dentry->d_name.name);
    #endif

    if(dentry != NULL && dentry->d_inode != NULL) {
        data = (char*)kmalloc(NL_MSG_MAX, GFP_KERNEL);
        if(data != NULL) {
            len = fuse_createMsg(OP_RMDIR, dir, dentry, NULL, data, NL_MSG_MAX, NULL, NULL);
            fuse_nl_send_msg(data, len);
            kfree(data);
        }
    }
    return 0;
}
int fuse_post_file_rename(struct inode *old_dir, struct dentry *old_dentry,
			 struct inode *new_dir, struct dentry *new_dentry) {
    char* data;
    size_t len = 0;

    if(!gEnable)
        return 0;

     if(g_pid == -1)
        return -1;

     #if DEBUG
     printk("====rename %s to %s\n", old_dentry->d_name.name, new_dentry->d_name.name);
     #endif

     if(old_dir != NULL && old_dentry != NULL && new_dir != NULL && new_dentry != NULL) {
        data = (char*)kmalloc(NL_RNAME_MSG_MAX, GFP_KERNEL);
        if(data != NULL) {
            len = fuse_createMsg(OP_RENAME, old_dentry->d_inode, NULL, NULL, data,NL_MSG_MAX, old_dentry, new_dentry);
            fuse_nl_send_msg(data, len);
            kfree(data);
        }
    }

     return 0;
}

int fuse_post_file_unlink(struct inode *dir, struct dentry *dentry) {
    char* data;
    size_t len = 0;

    if(!gEnable)
        return 0;

    if(g_pid == -1)
        return -1;

    #if DEBUG
    printk("unlink file:%s", dentry->d_name.name);
    #endif

    if(dentry != NULL && dentry->d_inode != NULL) {
        data = (char*)kmalloc(NL_MSG_MAX, GFP_KERNEL);
        if(data != NULL) {
            len = fuse_createMsg(OP_UNLINK, dentry->d_inode, dentry, NULL, data, NL_MSG_MAX, NULL, NULL);
            fuse_nl_send_msg(data, len);
            kfree(data);
        }
    }

    return 0;
}

int fuse_post_file_release(struct inode *inode, struct file *file) {
    char* data;
    struct fuse_file* ff;
    size_t len = 0;

    if(!gEnable)
        return 0;

    if(g_pid == -1)
        return -1;

    if(inode == NULL) {
        return -1;
    }

    if(inode != NULL) {
        if(!S_ISREG(inode->i_mode)) {
            return 0;
        }
    }

    #if DEBUG
    printk("post file release  is new:%lu\n", (inode->i_state & I_NEW));
    //show_stack(NULL, NULL);
    #endif

    if(file != NULL) {
        ff = (struct fuse_file*)file->private_data;
        if(file->f_path.dentry != NULL &&  ((ff->mask & FUSE_FS_MODIFY) )) {
            data = (char*)kmalloc(NL_MSG_MAX, GFP_KERNEL);
            if(data != NULL) {
                #if 1
                if(inode->i_ino == g_last_msg.ino && ((jiffies - g_last_msg.time) < HZ)) {
                    memset(&g_last_msg.data, 0, NL_MSG_MAX);
                    g_last_msg.data_len = fuse_createMsg(OP_CLOSE_WRITE, inode, file->f_path.dentry, &ff->creator, g_last_msg.data, NL_MSG_MAX, NULL, NULL);
                    g_last_msg.is_valid = true;

                    g_last_msg.ino = inode->i_ino;
                    g_last_msg.time = jiffies;
                } else {
                    if(g_last_msg.is_valid) {
                        fuse_nl_send_msg(g_last_msg.data, g_last_msg.data_len);
                        g_last_msg.is_valid = false;
                    }

                    g_last_msg.ino = inode->i_ino;
                    g_last_msg.time = jiffies;
                    len = fuse_createMsg(OP_CLOSE_WRITE, inode, file->f_path.dentry, &ff->creator, data, NL_MSG_MAX, NULL, NULL);
                    fuse_nl_send_msg(data, len);
                }
                #else
                len = fuse_createMsg(OP_CLOSE_WRITE, inode, file->f_path.dentry, &ff->creator, data, NL_MSG_MAX, NULL, NULL);
                fuse_nl_send_msg(data, len);
                #endif
                kfree(data);
            }
        }
    }
    return 0;
}


bool fuse_do_fileobserver_ioctl(struct file *file, unsigned int cmd, unsigned long arg,
		   unsigned int flags) {
       void __user *ubuf = (void __user *)arg;
       struct fuse_file *ff = file->private_data;

        #if DEBUG
        printk("FUSE ioctl be called  cmd:%u\n", cmd);
        #endif

        if(cmd == FUSE_FILEOBSERVER_IOCTL_CMD && ff != NULL) {
            if (copy_from_user(&ff->creator, ubuf, sizeof(struct fuse_file_creator))) {
                printk("copy from user error!\n");
                return false;
            } else {
                #if DEBUG
                printk("FUSE receive ioctl uid:%d, pid:%d\n", ff->creator.uid, ff->creator.pid);
                #endif
            }
            return true;
        }
        return false;
}
//Nubia File Observer End
