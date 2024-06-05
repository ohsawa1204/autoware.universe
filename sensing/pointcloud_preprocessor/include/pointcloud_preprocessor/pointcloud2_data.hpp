#include <sensor_msgs/msg/point_cloud2.h>

#define BLOCK_SIZE (2 * 1024 * 1024)
#define BLOCK_NUM 16

#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include <sys/stat.h>

int pointcloud2_data_semid __attribute__((weak));
int pointcloud2_data_shmid __attribute__((weak));
unsigned char *pointcloud2_data_shared_memory __attribute__((weak));

struct pointclouddata {
    bool used;
    uint8_t data[0];
};

union semun {
    int val;
    struct semid_ds *buf;
    unsigned short *array;
};

int get_semid(void)
{
    FILE *fp;
    const std::string file_path = "/tmp/semkey.dat";
    const int id = 50;
    key_t key;
    int semid;
    union semun arg;

    fp = fopen(file_path.c_str(), "w");
    fclose(fp);
    key = ftok(file_path.c_str(), id);
    if (key == -1)
        return -1;
    semid = semget(key, 1, 0666 | IPC_CREAT | IPC_EXCL);
    if (semid == -1)
        semid = semget(key, 1, 0);
    else {
        arg.val = 1;
        semctl(semid, 0, SETVAL, arg);
    }
    return semid;
}

int get_shmid(void)
{
    FILE *fp;
    const std::string file_path = "/tmp/shmkey.dat";
    const int id = 50;
    key_t key;
    const int size = BLOCK_SIZE * BLOCK_NUM;
    int shmid;
    
    fp = fopen(file_path.c_str(), "w");
    fclose(fp);
    key = ftok(file_path.c_str(), id);
    if (key == -1)
        return -1;
    shmid = shmget(key, size, IPC_CREAT | IPC_EXCL | S_IRUSR | S_IWUSR);
    if (shmid == -1)
        shmid = shmget(key, 0, 0);
    return shmid;
}

unsigned char *attach_shared_memory(int shmid)
{
    unsigned char *shared_memory = reinterpret_cast<unsigned char*>(shmat(shmid, 0, 0));
    return shared_memory;
}

void init_shared_memory(void)
{
    int i;
    struct pointclouddata *ptr;
    for (i = 0; i < BLOCK_NUM; i++) {
        ptr = (struct pointclouddata *)(pointcloud2_data_shared_memory + BLOCK_SIZE * i);
        ptr->used = false;
    }
}

int init_IPC(void)
{
    pointcloud2_data_semid = get_semid();
    if (pointcloud2_data_semid == -1)
        return -1;
    pointcloud2_data_shmid = get_shmid();
    if (pointcloud2_data_shmid == -1)
        return -1;
    pointcloud2_data_shared_memory = attach_shared_memory(pointcloud2_data_shmid);
    return 0;
}

uint64_t get_free_shared_memory_block(void)
{
    int i;
    struct pointclouddata *ptr;
    struct sembuf sop;
    sop.sem_num =  0;
    sop.sem_op  = -1;
    sop.sem_flg =  0;

    semop(pointcloud2_data_semid, &sop, 1);
    sop.sem_op  = 1;

    for (i = 0; i < BLOCK_NUM; i++) {
        ptr = (struct pointclouddata *)(pointcloud2_data_shared_memory + BLOCK_SIZE * i);
        if (!ptr->used) {
            ptr->used = true;
            semop(pointcloud2_data_semid, &sop, 1);
            return BLOCK_SIZE * i;
        }
    }
    semop(pointcloud2_data_semid, &sop, 1);
    return (uint64_t)-1;
}

void release_shared_memory_block(uint64_t offset)
{
    struct pointclouddata *ptr = (struct pointclouddata *)(pointcloud2_data_shared_memory + offset);
    ptr->used = false;
}

void copy_to_metadata_msg(sensor_msgs::msg::PointCloud2 &pc2, pointcloud2_metadata_msgs::msg::PointCloud2MetaData &pc2_metadata)
{
    pc2_metadata.header = pc2.header;
    pc2_metadata.height = pc2.height;
    pc2_metadata.width = pc2.width;
    pc2_metadata.fields = pc2.fields;
    pc2_metadata.is_bigendian = pc2.is_bigendian;
    pc2_metadata.point_step = pc2.point_step;
    pc2_metadata.row_step = pc2.row_step;
    pc2_metadata.is_dense = pc2.is_dense;
    pc2_metadata.data_size = pc2.data.size();
    pc2_metadata.data_length = pc2.data.size() * sizeof(pc2.data[0]);
}

void copy_from_metadata_msg(sensor_msgs::msg::PointCloud2 &pc2, const pointcloud2_metadata_msgs::msg::PointCloud2MetaData::ConstSharedPtr &pc2_metadata)
{
    pc2.header = pc2_metadata->header;
    pc2.height = pc2_metadata->height;
    pc2.width = pc2_metadata->width;
    pc2.fields = pc2_metadata->fields;
    pc2.is_bigendian = pc2_metadata->is_bigendian;
    pc2.point_step = pc2_metadata->point_step;
    pc2.row_step = pc2_metadata->row_step;
    pc2.is_dense = pc2_metadata->is_dense;
}

void copy_to_shared_memory(uint64_t offset, sensor_msgs::msg::PointCloud2 &pointcloud2, size_t data_length)
{
    struct pointclouddata *ptr = (struct pointclouddata *)(pointcloud2_data_shared_memory + offset);
    memcpy(ptr->data, &pointcloud2.data[0], data_length);
}

void copy_from_shared_memory(uint64_t offset, sensor_msgs::msg::PointCloud2 &pointcloud2, size_t data_size, size_t data_length)
{
    struct pointclouddata *ptr = (struct pointclouddata *)(pointcloud2_data_shared_memory + offset);
    pointcloud2.data.resize(data_size);
    memcpy(&pointcloud2.data[0], ptr->data, data_length);
}
