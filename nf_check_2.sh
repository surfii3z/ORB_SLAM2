SEQ=00
for NF in 1750
do
    mkdir -p nf_results/${SEQ}_${NF}
    for ITER in 0 1 2 3 4
    do
        ./Examples/RGB-D/rgbd_kitti Vocabulary/ORBvoc.bin Examples/RGB-D/rgbd_kitti_00_02_${NF}.yaml /data/datasets/KITTI_odom_color/${SEQ}
        mv KeyFrameTrajectory.txt nf_results/${SEQ}_${NF}/${SEQ}_${NF}_${ITER}.txt
    done
    
done


SEQ=02
for NF in 2250 2500 2750
do
    mkdir -p nf_results/${SEQ}_${NF}
    for ITER in 0 1 2 3 4
    do
        ./Examples/RGB-D/rgbd_kitti Vocabulary/ORBvoc.bin Examples/RGB-D/rgbd_kitti_00_02_${NF}.yaml /data/datasets/KITTI_odom_color/${SEQ}
        mv KeyFrameTrajectory.txt nf_results/${SEQ}_${NF}/${SEQ}_${NF}_${ITER}.txt
    done
    
done

SEQ=03
for NF in 500 750 1250 1500 1750 2250 2500 2750
do
    mkdir -p nf_results/${SEQ}_${NF}
    for ITER in 0 1 2 3 4
    do
        ./Examples/RGB-D/rgbd_kitti Vocabulary/ORBvoc.bin Examples/RGB-D/rgbd_kitti_03_${NF}.yaml /data/datasets/KITTI_odom_color/${SEQ}
        mv KeyFrameTrajectory.txt nf_results/${SEQ}_${NF}/${SEQ}_${NF}_${ITER}.txt
    done
    
done