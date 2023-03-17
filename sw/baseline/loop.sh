#names=(fr1_xyz fr1_desk fr1_desk2 fr2_xyz fr2_rpy)
dirnames=(1_xyz 1_desk 1_room 1_desk2 2_xyz 2_rpy 2_desk)
#num=(1 1 1 2 2)

rm -f results.txt
touch results.txt

for dirname in ${dirnames[@]}; do
    echo $i
    name=fr${dirname}
    num=${dirname::1}
    dirname=rgbd_dataset_freiburg${dirname}
    cmd="../Examples/RGB-D/rgbd_tum ../Vocabulary/ORBvoc.txt ../Examples/RGB-D/TUM$num.yaml ../sequences/$dirname/  ../sequences/$dirname/associations.txt"
    echo $cmd
    echo $name
    $cmd
    ../rgbd_benchmark_tools/scripts/evaluate_ate.py CameraTrajectory.txt ../sequences/$dirname/groundtruth.txt > ${name}_ate.txt
    mv CameraTrajectory.txt ${name}_cam.txt
    mv KeyFrameTrajectory.txt ${name}_kf.txt

    echo -en "$name\t\t\t" >> results.txt
    cat ${name}_ate.txt >> results.txt
    echo -n "" >> results.txt
done

