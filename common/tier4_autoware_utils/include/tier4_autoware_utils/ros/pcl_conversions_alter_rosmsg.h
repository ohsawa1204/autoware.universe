#ifndef PCL_CONVERSIONS_ALTER_ROSMSG_H__
#define PCL_CONVERSIONS_ALTER_ROSMSG_H__

#include <pcl_conversions/pcl_conversions.h>

namespace pcl {
  template<typename PointT> void
  alter_toPCLPointCloud2 (const pcl::PointCloud<PointT>& cloud, pcl::PCLPointCloud2& msg)
  {
    // Ease the user's burden on specifying width/height for unorganized datasets
    if (cloud.width == 0 && cloud.height == 0)
    {
      msg.width  = cloud.size ();
      msg.height = 1;
    }
    else
    {
      assert (cloud.size () == cloud.width * cloud.height);
      msg.height = cloud.height;
      msg.width  = cloud.width;
    }

    // Fill point cloud binary data (padding and all)
    std::size_t data_size = sizeof (PointT) * cloud.size ();
    //msg.data.resize (data_size);
    if (data_size)
    {
      //memcpy(&msg.data[0], &cloud[0], data_size);
      msg.data.swap(*reinterpret_cast<std::vector<unsigned char> *>((const_cast<std::vector<PointT, Eigen::aligned_allocator<PointT>> *>(&cloud.points))));
    }

    // Fill fields metadata
    msg.fields.clear ();
    for_each_type<typename traits::fieldList<PointT>::type> (detail::FieldAdder<PointT>(msg.fields));

    msg.header     = cloud.header;
    msg.point_step = sizeof (PointT);
    msg.row_step   = (sizeof (PointT) * msg.width);
    msg.is_dense   = cloud.is_dense;
    /// @todo msg.is_bigendian = ?;
  }

  template <typename PointT> void
  alter_fromPCLPointCloud2 (sensor_msgs::msg::PointCloud2 &pc2, const pcl::PCLPointCloud2& msg, pcl::PointCloud<PointT>& cloud,
              const MsgFieldMap& field_map)
  {
    // Copy info fields
    cloud.header   = msg.header;
    cloud.width    = msg.width;
    cloud.height   = msg.height;
    cloud.is_dense = msg.is_dense == 1;

    // Copy point data
    cloud.resize (msg.width * msg.height);
    std::uint8_t* cloud_data = reinterpret_cast<std::uint8_t*>(&cloud[0]);

    // Check if we can copy adjacent points in a single memcpy.  We can do so if there
    // is exactly one field to copy and it is the same size as the source and destination
    // point types.
    if (field_map.size() == 1 &&
        field_map[0].serialized_offset == 0 &&
        field_map[0].struct_offset == 0 &&
        field_map[0].size == msg.point_step &&
        field_map[0].size == sizeof(PointT))
    {
      const auto cloud_row_step = (sizeof (PointT) * cloud.width);
      const std::uint8_t* msg_data = &pc2.data[0];
      // Should usually be able to copy all rows at once
      if (msg.row_step == cloud_row_step)
      {
        memcpy (cloud_data, msg_data, pc2.data.size ());
      }
      else
      {
        for (uindex_t i = 0; i < msg.height; ++i, cloud_data += cloud_row_step, msg_data += msg.row_step)
          memcpy (cloud_data, msg_data, cloud_row_step);
      }

    }
    else
    {
      // If not, memcpy each group of contiguous fields separately
      for (uindex_t row = 0; row < msg.height; ++row)
      {
        const std::uint8_t* row_data = &pc2.data[row * msg.row_step];
        for (uindex_t col = 0; col < msg.width; ++col)
        {
          const std::uint8_t* msg_data = row_data + col * msg.point_step;
          for (const detail::FieldMapping& mapping : field_map)
          {
            memcpy (cloud_data + mapping.struct_offset, msg_data + mapping.serialized_offset, mapping.size);
          }
          cloud_data += sizeof (PointT);
        }
      }
    }
  }

  template<typename PointT> void
  alter_fromPCLPointCloud2 (sensor_msgs::msg::PointCloud2 &pc2, const pcl::PCLPointCloud2& msg, pcl::PointCloud<PointT>& cloud)
  {
    MsgFieldMap field_map;
    createMapping<PointT> (msg.fields, field_map);
    alter_fromPCLPointCloud2 (pc2, msg, cloud, field_map);
  }

    template<typename T>
  void alter_toROSMsg(const pcl::PointCloud<T> &pcl_cloud, sensor_msgs::msg::PointCloud2 &cloud)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    alter_toPCLPointCloud2(pcl_cloud, pcl_pc2);
    pcl_conversions::moveFromPCL(pcl_pc2, cloud);
  }

  template<typename T>
  void alter_fromROSMsg(const sensor_msgs::msg::PointCloud2 &cloud, pcl::PointCloud<T> &pcl_cloud)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::copyPointCloud2MetaData(cloud, pcl_pc2);
    alter_fromPCLPointCloud2(const_cast<sensor_msgs::msg::PointCloud2&>(cloud), pcl_pc2, pcl_cloud);
  }
}

#endif /* PCL_CONVERSIONS_ALTER_ROSMSG_H__ */
