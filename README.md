# A. �C���X�g�[���菇
~~~
cd ~/catkin_ws/src/
git clone https://github.com/YOODS/rqt_param_manager
pip install pyyaml
~~~

# B. �G���h���[�U�ɂ��g����

## B-1. �N��
�N���C�A���g�Ƃ���RoVI���g�p�����F  
��RoVI commit 45c7c96�ɂ��A�����_�ł͎g���Ȃ��Ȃ��Ă���̂Œ��ӁB45c7c96�̈�O��10fb908�ɖ߂��Ύg����B  
~~~
roscd rovi
git pull origin ramiel
# RoVI�̋N�� (vga��sxga�֓ǂݑւ��\�B�ȉ����l�B)
roslaunch rovi ycam3vga.launch
# �{�c�[���̋N��
roslaunch rovi ycam3vga_livecamera_pm.launch
roslaunch rovi ycam3vga_pshift_pm.launch
~~~

## B-2. ����A�m�F
rviz�̃��C�u��ʂ� rosservice call /rovi/pshift_genpc �ł̓_�Q�����ŁArqt_param_manager�������Ă���̂��m�F�ł���B


# C. �{�c�[���̃N���C�A���g�̍쐬�菇
RoVI�ȊO�̃p�b�P�[�W�i���� xxx �Ƃ���j���N���C�A���g�ɂ���ꍇ�̎菇�͈ȉ��B  
(�O��Ƃ��āAROS Parameter Namespace �� xxx �Ƃ���B)  
(��Ƃ��āA�p�����[�^�O���[�vA, B��2�Ƃ���B)  

1. ~/catkin_ws/src/xxx/pmjson/ ��PM�ݒ�t�@�C�� (A_pm.json, B_pm.json) ��p�ӂ���B(RoVI�̂��R�s�[���ĕҏW����΂悢�B)

�@dumpYaml�̒l��params�̒��g��K�؂ɏ���������B

�@�@YOODS�Г��K��Ƃ��āA  
�@�@dumpYaml�̒l�� yaml/*_ld.yaml �� * ����������������΂悢�B  
�@�@�iyaml/A_ld.yaml, yaml/B_ld.yaml�ցB�j  

�@�@params�̒��g�̓p�����[�^�O���[�v�Ƃ��̍\��Parameter path�Ō��܂�B  
�@�@�iparamName�̒l�̑O�ɁA�����I�ɂ� /xxx/ ���t�����Ƃɒ��ӁB�j

2. ��LdumpYaml�̒l�t�@�C�� (~/catkin_ws/src/xxx/yaml/{A,B}_ld.yaml) ��p�ӂ���B(RoVI�̂��R�s�[���ĕҏW����΂悢�B)

�@���g���K�؂ɁB

�@�@�V�K���Ɗ֌W�Ȃ����ARoVI�̂悤�Ɋ����� yaml/ycam3vga.yaml ������Ȃ�A���̒�����  
�@�@~/catkin_ws/src/xxx/yaml/{A,B}_ld.yaml  
�@�@�Ƃ��Ԃ�p�����[�^���폜���Ă������Ƃ��Y�ꂸ�ɁB  
�@�@�i�폜���Ă����Ȃ��Ǝ��̎菇���ŏ㏑��load����Ă��܂��̂ŁB�j

3. �N���C�A���g��launch�i�₻�������sh�j�̒��ŁA  
~~~
ROS_NAMESPACE=/xxx rosparam load yaml/A_ld.yaml
ROS_NAMESPACE=/xxx rosparam load yaml/B_ld.yaml
ROS_NAMESPACE=/xxx rosparam load yaml/CLIENT_BASIC.yaml
~~~
�@�����̂��Ƃ�����B

4. �p�����[�^�ҏW�c�[����launch�t�@�C�� (~/catkin_ws/src/xxx/launch/{A,B}_pm.launch) ��p�ӂ���B(RoVI�̂��R�s�[���ĕҏW����΂悢�B)

�@�Ⴆ��  
�@https://github.com/YOODS/rovi/blob/ramiel/launch/ycam3vga_livecamera_pm.launch  
�@���R�s�[���āA  
~~~
<launch>
  <env name="PMCLIENT_PKG_PATH" value="$(find rovi)" />
  <node ns="rovi" name="ycam3vga_livecamera_pm" pkg="rqt_param_manager" type="rqt_param_manager" args="conffile:=$(find rovi)/pmjson/ycam3vga_livecamera_pm.json" />
</launch>
~~~
�@�́urovi�v3�ӏ����uxxx�v�ɏ��������A  
�@&lt;node&gt;��name�̒l��A_pm�ɏ��������A  
�@conffile�̒l��ycam3vga_livecamera_pm.json��A_pm.json�ɏ��������B

5. ��͈ȉ��Ŏ��s�ł���B
- �N���C�A���g��launch���s
- �p�����[�^�ҏW�c�[��A��launch���s
- �p�����[�^�ҏW�c�[��B��launch���s

