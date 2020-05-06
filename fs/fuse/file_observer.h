#ifndef _FILE_OBSERVER_H
#define _FILE_OBSERVER_H

#include <linux/errno.h>

#ifdef __KERNEL__

#define FUSE_RELEASE_PRIVATE    0x00000001	/* private data need be released */
//#define FUSE_FS_CREATE	    0x00000002	/* File was created */
#define FUSE_FS_MODIFY    0x00000004	/* File was modified */
#define FUSE_FILEOBSERVER_IOCTL_CMD 1000
#define USER_NETLINK_CMD	26
#define MAXMSGLEN 			1024

#define EVENT_CREATE "create"
#define EVENT_CLOSE_WRITE "close-write"
#define EVENT_UNLINK "unlink"
#define EVENT_RMDIR "rmdir"
#define EVENT_MKDIR "mkdir"
#define EVENT_RENAME "rename"

#define INODE_INUM_LEN_MAX 64
#define PROCESS_NAME_MAX 128
#define NL_MSG_MAX (PATH_MAX+PROCESS_NAME_MAX+128)
#define NL_RNAME_MSG_MAX (PATH_MAX*2+PROCESS_NAME_MAX+128)

typedef enum type_e {
        NL_TYPE_NONE,
	NL_TYPE_SET_DEST_ADDR,
	NL_TYPE_SET_ENABLE,
        NL_TYPE_SET_DISABLE
} netlink_type;


typedef enum FO_OP_e {
	OP_CREATE,
	OP_RENAME,
	OP_CLOSE_WRITE,
	OP_UNLINK,
	OP_MKDIR,
	OP_RMDIR,
} FO_OP;

typedef enum FO_FRAGEMENT_e {
	FRAGEMENT_INO,
	FRAGEMENT_SIZE,
	FRAGEMENT_UID,
	FRAGEMENT_PID,
	FRAGEMENT_CUID,
	FRAGEMENT_CPID,
	FRAGEMENT_TNAME,
	FRAGEMENT_PNAME,
	FRAGEMENT_PATH,
	FRAGEMENT_OLDPATH,
	FRAGEMENT_NEWPATH,
} FO_FRAGEMENT;

typedef struct FO_FragMent_t {
    int fm;
    int offset;
    int len;
}FO_FragMent;

typedef struct FO_Header_t {
    char flag[4];
    int op;
    int fmCnt;
}FO_Header;


typedef struct Last_Msg_t {
    unsigned long ino;
    long time;
    unsigned char data[NL_MSG_MAX];
    int volatile data_len;
    bool volatile is_valid;
}Last_Msg;



int fuse_init_file_observer(void);
int fuse_exit_file_observer(void);
int fuse_post_file_release(struct inode *inode, struct file *file);
//int post_file_create(struct inode *inode, struct dentry *dentry) ;
//int fuse_post_file_open(struct file *file);
int fuse_post_file_write(struct file *file) ;
int fuse_post_file_create(struct dentry *dentry) ;
int fuse_post_file_unlink(struct inode *dir, struct dentry *dentry) ;
int fuse_post_file_mkdir(struct inode *dir, struct dentry *dentry) ;
int fuse_post_file_rmdir(struct inode *dir, struct dentry *dentry) ;
int fuse_post_file_rename(struct inode *old_dir, struct dentry *old_dentry,
			 struct inode *new_dir, struct dentry *new_dentry) ;
bool fuse_do_fileobserver_ioctl(struct file *file, unsigned int cmd, unsigned long arg,
		   unsigned int flags) ;


#endif /* __KERNEL__ */
#endif /* _LINUX_MM_H */
