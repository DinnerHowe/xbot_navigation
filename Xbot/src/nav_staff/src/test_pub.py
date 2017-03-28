#!/usr/bin/env python
# coding=utf-8
"""
test plan 算法库的测试程序

Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
import numpy
from PlanAlgrithmsLib import Data
from PlanAlgrithmsLib import maplib
#switch modles
class tester1():
    def __init__(self):
        rospy.Timer(rospy.Duration(0.1), self.pubCB)
        rospy.Timer(rospy.Duration(0.1), self.switchCB)
        rospy.spin()

    def pubCB(self,event):
        pub1 = rospy.Publisher('test/1', String, queue_size=1)
        pub2 = rospy.Publisher('test/2', String, queue_size=1)
        pub1.publish('1')
        pub2.publish('2')

    def switchCB(self, event):
        switcher = rospy.Publisher('/move_base/switch', String, queue_size=1)
        string = raw_input('FixedModule / OnePathModule:')
        switcher.publish(string)

#数据融合
class tester2():
    def __init__(self):
        inf = numpy.inf
        ldata = [0.33500000834465027, 0.33399999141693115, 0.3330000042915344, inf, 0.3319999873638153, 0.33149999380111694, 0.33149999380111694, 0.3305000066757202, 0.3312000036239624, inf, inf, inf, inf, inf, 0.3400000035762787, inf, 0.33219999074935913, inf, inf, inf, 0.34630000591278076, inf, 0.34119999408721924, 0.34299999475479126, 0.34450000524520874, 0.3467999994754791, 0.34929999709129333, inf, 0.35199999809265137, 0.3544999957084656, 0.3569999933242798, inf, 0.3598000109195709, 0.3634999990463257, 0.36730000376701355, 0.3702999949455261, 0.3743000030517578, 0.3781999945640564, 0.3824999928474426, inf, 0.3871999979019165, 0.39169999957084656, 0.39750000834465027, 0.4016999900341034, 0.4090000092983246, 0.41429999470710754, inf, 0.4212999939918518, 0.42800000309944153, 0.4343000054359436, 0.4424999952316284, 0.4507000148296356, inf, 0.45899999141693115, 0.4693000018596649, 0.47850000858306885, 0.489300012588501, 0.49950000643730164, 0.48579999804496765, inf, 0.47850000858306885, 0.4708000123500824, 0.46399998664855957, 0.45750001072883606, inf, 0.47049999237060547, 0.4812999963760376, 0.4932999908924103, inf, inf, 2.129199981689453, inf, 7.210299968719482, 7.052499771118164, 7.042699813842773, 7.020999908447266, inf, 6.876200199127197, inf, 6.867300033569336, 6.862500190734863, inf, 6.740799903869629, 6.621200084686279, 6.497499942779541, 6.495500087738037, 6.499499797821045, inf, 6.4770002365112305, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 6.368500232696533, 6.4832000732421875, 6.366499900817871, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 2.941499948501587, 3.077699899673462, inf, inf, inf, inf, 2.2207000255584717, 2.2255001068115234, 2.2325000762939453, inf, 2.1419999599456787, 2.072499990463257, 1.9982999563217163, 1.9305000305175781, inf, 1.7170000076293945, 1.7408000230789185, 1.8795000314712524, 1.9119999408721924, inf, 1.9470000267028809, 1.980299949645996, 2.0297000408172607, 2.0739998817443848, 2.114500045776367, 2.8965001106262207, inf, 2.9702999591827393, 1.6395000219345093, 1.576799988746643, 2.384500026702881, inf, 2.315200090408325, 2.4177000522613525, inf, inf, 1.6734999418258667, inf, inf, inf, inf, inf, inf, inf, inf, 1.1109999418258667, 1.0987000465393066, 1.0915000438690186, 1.1318000555038452, 1.1506999731063843, inf, 1.1414999961853027, 1.132699966430664, 1.1269999742507935, 1.1217999458312988, inf, 1.1174999475479126, 1.1109999418258667, 1.1066999435424805, 1.1019999980926514, 1.0980000495910645, inf, 1.0946999788284302, 1.090999960899353, 1.0908000469207764, 1.087499976158142, inf, inf, inf, 0.8015000224113464, 0.7926999926567078, inf, inf, inf, inf, 0.7710000276565552, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 1.6065000295639038, 1.6065000295639038, 1.5978000164031982, inf, inf, 2.13319993019104, inf, 2.1612000465393066, 2.1796998977661133, 2.210200071334839, 2.177500009536743, inf, 2.1052000522613525, 2.056999921798706, 1.975000023841858, inf, inf, inf, inf, inf, inf, 1.2747999429702759, 1.2799999713897705, 1.2929999828338623, inf, inf, inf, inf, 1.514299988746643, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 2.014699935913086, inf, 3.939500093460083, 2.640500068664551, 4.249000072479248, inf, inf, inf, inf, inf, 10.224800109863281, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 3.0209999084472656, 2.923799991607666, inf, 1.4378000497817993, 1.4122999906539917, inf, inf, inf, inf, inf, 0.7365000247955322, 0.7450000047683716, inf, 0.6729000210762024, inf, 0.6381999850273132, 0.6182000041007996, inf, inf, 0.6585000157356262, 0.6736999750137329, inf, inf, inf, inf, inf, inf, inf, 1.2304999828338623, 1.2404999732971191, 1.253000020980835, 1.2675000429153442, 1.2848000526428223, inf, 1.297700047492981, 1.315000057220459, 1.3313000202178955, 1.3504999876022339, 1.3734999895095825, inf, 1.392199993133545, 1.4134999513626099, 1.4363000392913818, 1.4702999591827393, 1.4930000305175781, inf, 1.5195000171661377, 1.555999994277954, 1.587499976158142, 1.6180000305175781, 1.5844999551773071, inf, 1.5535000562667847, 1.5195000171661377, inf, inf, inf, inf, 0.3179999887943268, 0.31450000405311584, inf, 0.31029999256134033, 0.3089999854564667, 0.3181999921798706, 0.32749998569488525, 0.33869999647140503, 0.35179999470710754, inf, inf, 1.9600000381469727, 1.9366999864578247, inf, inf, inf, inf, inf, 0.33799999952316284, 0.3375000059604645, 0.335999995470047, inf, 0.34049999713897705, 0.33820000290870667, 0.3370000123977661, 0.33570000529289246, inf]
        # ldata = [inf for i in range(360)]
        # ldata[0] = 1.0
        ldata[1] = 1.0
        ldata[10] = 1.0
        ldata[-1] = 1.0
        lidar_data = LaserScan()
        lidar_data.angle_max = numpy.pi
        lidar_data.angle_min = -numpy.pi
        lidar_data.angle_increment = -numpy.pi/180.0
        lidar_data.range_max = 6.0
        lidar_data.range_min = 0.15
        seq = 0
        lidar_data.header.seq = seq
        lidar_data.header.frame_id = 'laser'
        pub1 = rospy.Publisher('/rplidar_scan', LaserScan, queue_size=1)
        lidar_data.ranges = ldata

        rate = rospy.Rate(7)

        nan = numpy.nan
        adata = [nan, nan, nan, nan, nan, nan, 1.7486488819122314, 1.7473496198654175, 1.7460535764694214, 1.760736107826233, 1.7581361532211304, 1.7648110389709473, 1.7714775800704956, 1.7701771259307861, 1.7768324613571167, 1.775532841682434, 1.7820119857788086, 1.7886449098587036, 1.7873460054397583, 1.7939683198928833, 1.792670488357544, 1.790084719657898, 1.7887970209121704, 1.8044184446334839, 1.8031253814697266, 1.810839295387268, 1.8174192905426025, 1.8251152038574219, 1.8238176107406616, 1.8302044868469238, 1.8289111852645874, 1.8354605436325073, 1.8431212902069092, 1.8418269157409668, 1.8405359983444214, 1.8392488956451416, 1.8634495735168457, 1.8621547222137451, 1.8708860874176025, 1.8695917129516602, 1.8771977424621582, 1.8847954273223877, 1.883500099182129, 1.8909021615982056, 1.8984777927398682, 1.89718496799469, 1.905856728553772, 1.9145190715789795, 1.9132243394851685, 1.9207698106765747, 1.919477105140686, 1.9169034957885742, 1.9156228303909302, 1.9143459796905518, 1.913073182106018, 1.921698808670044, 1.9303152561187744, 1.9389228820800781, 1.9376466274261475, 1.9473392963409424, 1.9460642337799072, 1.9645038843154907, 1.972885012626648, 1.9814428091049194, 1.9801623821258545, 1.9788862466812134, 1.9885222911834717, 1.9872477054595947, 1.9968714714050293, 2.0173728466033936, 2.0160906314849854, 2.014812707901001, 2.0244054794311523, 2.0327110290527344, 2.042283058166504, 2.0410070419311523, 2.0397355556488037, 2.038468837738037, 2.049107313156128, 2.047842264175415, 2.046581745147705, 2.0226478576660156, 2.0786101818084717, 2.0773425102233887, 2.0868587493896484, 2.0974438190460205, 2.109096050262451, 2.107825994491577, 2.106560707092285, 2.117121696472168, 2.139289379119873, 2.138021945953369, 2.149625062942505, 2.148359775543213, 2.147099733352661, 2.1458446979522705, 2.144594430923462, 2.1433494091033936, 2.1421093940734863, 2.1408743858337402, 2.1396446228027344, 2.1384196281433105, 2.137199878692627, 2.1359853744506836, 2.1347756385803223, 2.133571147918701, 2.1323719024658203, 2.1311776638031006, 2.1172404289245605, 2.1288046836853027, 2.114891767501831, 2.113725185394287, 2.1125638484954834, 2.111407518386841, 2.122962236404419, 2.109110116958618, 2.1079695224761963, 2.1068336963653564, 1.5805981159210205, 1.5514717102050781, 1.5443201065063477, 1.5371791124343872, 1.5363712310791016, 1.5292479991912842, 1.5210824012756348, 1.5076676607131958, 1.5005793571472168, 1.4871951341629028, 1.4864333868026733, 1.4741261005401611, 1.460784673690796, 1.4548019170761108, 1.4425382614135742, 1.4355287551879883, 1.423292636871338, 1.422586441040039, 1.4218838214874268, 1.4107272624969482, 1.4100371599197388, 1.4093506336212158, 1.402402400970459, 1.3965075016021729, 1.3958373069763184, 1.389957070350647, 1.3892966508865356, 1.3834311962127686, 1.3827805519104004, 1.3758889436721802, 1.3752484321594238, 1.3694125413894653, nan, 1.368781566619873, 1.3629599809646606, 1.362338662147522, 1.3617209196090698, 1.3455454111099243, 1.3449418544769287, 1.3443418741226196, 1.3541059494018555, 1.3586864471435547, 1.3580904006958008, 1.357498049736023, 1.3569092750549316, 1.3563241958618164, 1.3557426929473877, 1.355164885520935, 1.3545907735824585, 1.354020357131958, 1.353453516960144, 1.3528903722763062, 1.3523309230804443, 1.3517751693725586, 1.351223111152649, 1.3558220863342285, 1.3552753925323486, 1.3660473823547363, 1.3655035495758057, 1.364963412284851, 1.364427089691162, 1.3638945817947388, 1.3633657693862915, 1.3628407716751099, 1.3623195886611938, 1.361802101135254, nan, 1.3612884283065796, 1.3659019470214844, 1.3653939962387085, 1.3648897409439087, 1.364389419555664, 1.363892912864685, 1.3634002208709717, 1.362911343574524, 1.3624262809753418, 1.3619450330734253, 1.3665745258331299, 1.3609941005706787, 1.3707311153411865, 1.370261788368225, 1.3697963953018188, 1.3591386079788208, 1.3586844205856323, 1.3633288145065308, 1.3740850687026978, 1.4652808904647827, nan, 1.4648072719573975, 1.4643378257751465, 1.4577689170837402, 1.4573098421096802, 1.4507551193237305, 1.4503062963485718, 1.44376540184021, 1.4433268308639526, 1.44289231300354, 1.4424620866775513, 1.4359471797943115, 1.435526967048645, 1.4351110458374023, 1.4346991777420044, 1.4342916011810303, nan, 1.428821325302124, 1.428423523902893, 1.4219532012939453, 1.4154902696609497, 1.4151082038879395, 1.414730191230774, 1.408286213874817, 1.4079182147979736, 1.407554268836975, 1.4071944952011108, 1.4017856121063232, 1.4064874649047852, 1.4061400890350342, 1.4057968854904175, nan, 1.4115159511566162, 1.4172362089157104, 1.4169026613235474, 1.4226269721984863, 1.4273440837860107, 1.42702054977417, 1.4267014265060425, 1.4263863563537598, 1.4210364818572998, 1.3975560665130615, 1.3912155628204346, 1.3858888149261475, 1.3856031894683838, nan, 1.3792810440063477, 1.3790048360824585, 1.3787329196929932, 1.3784650564193726, 1.384237289428711, 1.3839764595031738, 1.3837199211120605, 1.3834675550460815, 1.3882454633712769, 1.3940309286117554, 1.3937890529632568, 1.3985750675201416, nan, 1.3983407020568848, 1.404137134552002, 1.4039102792739868, 1.4097121953964233, 1.4155163764953613, 1.4203191995620728, 1.4201068878173828, 1.4259196519851685, 1.4257149696350098, 1.4255146980285645, 1.4313369989395142, nan, 1.4431793689727783, 1.442989706993103, 1.4488202333450317, 1.448638677597046, 1.4484614133834839, 1.442274808883667, 1.4240682125091553, 1.423906922340393, 1.4177383184432983, 1.417586326599121, 1.4174387454986572, nan, 1.4172954559326172, 1.417156457901001, 1.4170217514038086, 1.4168914556503296, 1.4167654514312744, 1.4166438579559326, 1.411521077156067, 1.411408543586731, 1.4113003015518188, 1.4111963510513306, nan, 1.4050921201705933, 1.4049972295761108, 1.4049066305160522, 1.404820442199707, 1.404738426208496, 1.3986579179763794, 1.3985849618911743, 1.3985161781311035, 1.3984516859054565, 0.9782739281654358, 0.9782348275184631, nan, 0.978198766708374, 0.9781656861305237, 0.9781356453895569, 0.9781085848808289, 0.9750843048095703, 0.9750633239746094, 0.975045382976532, 0.9750303626060486, 0.9750183820724487, 0.9750093817710876, nan, 0.9730033278465271, 0.9750003814697266, 0.9750003814697266, 0.9750034213066101, 0.9730093479156494, 0.9730182886123657, 0.9730302691459656, 0.9730452299118042, 0.9730631709098816, 0.9730840921401978, 0.9481052756309509, nan, 0.9541323184967041, 0.9541616439819336, 0.9641959071159363, 0.9672322273254395, 0.9672708511352539, 0.9703134298324585, 0.9703581929206848, 0.9704058766365051, 0.9704565405845642, 0.9705101847648621, nan, 0.9705668091773987, 0.9706264138221741, 0.9706889986991882, 0.9707545638084412, 0.9708230495452881, 0.9708945751190186, 0.9709690809249878, 0.971046507358551, 0.971126914024353, 0.9682065844535828, nan, 0.9682926535606384, 0.9683817028999329, 0.9684737324714661, 0.9685686826705933, 0.9656614661216736, 0.9657620191574097, 0.9658655524253845, 0.9659720063209534, 0.966081440448761, 0.9661937952041626, 0.9612971544265747, nan, 0.9694349765777588, 0.9695565104484558, 0.9666727185249329, 0.966799795627594, 0.9669297933578491, 0.9650564193725586, 0.9651920199394226, 0.96232008934021, 0.9624611139297485, 0.9626050591468811, 0.9627518653869629, nan, 0.9629016518592834, 0.9630542993545532, 0.9632099270820618, 0.9633684158325195, 0.9635297656059265, 0.9636940956115723, 0.9638612866401672, 0.9640313982963562, 0.9642044305801392, 0.9643803834915161, 0.9645591974258423, 0.9647408723831177, nan, 0.961906909942627, 0.9620938301086426, 0.9602705240249634, 0.9624763131141663, 0.9626718759536743, 0.9628703594207764, 0.9630716443061829, 0.9632758498191833, 0.9634829163551331, 0.963692843914032, 0.9639056324958801, 0.9641213417053223, nan, 0.9643398523330688, 0.9645612835884094, 0.9647855162620544, 0.9650126695632935, 0.9652426242828369, 0.9654754400253296, 0.9636908769607544, 0.9639288783073425, 0.9641698002815247, 0.9644135236740112, 0.9616265296936035, 0.9618751406669617, 0.9621265530586243, nan, 0.9593449831008911, 0.9575767517089844, 0.9578352570533752, 0.9601222276687622, 0.9603869318962097, 0.9586277008056641, 0.9588974118232727, 0.9611977934837341, 0.9614737033843994, 0.9617524147033691, 0.9620338678359985, 0.9623181223869324, 0.9626051783561707, 0.9628950357437134, 0.9611555933952332, nan, 0.9584013819694519, 0.9617478847503662, 0.9620481729507446, 0.9623512625694275, 0.96265709400177, 0.962965726852417, 0.9632770419120789, 0.9635911583900452, 0.9639080166816711, 0.9642276763916016, 0.9645500183105469, 0.9648751020431519, 0.9652029275894165, 0.9655335545539856, 0.9658668637275696, 0.9662029147148132, 0.9634765386581421, 0.9638170003890991, nan, 0.964160144329071, 0.9645059704780579, 0.9648545384407043, 0.9652057886123657, 0.9655597805976868, 0.9659164547920227, 0.9662758111953735, 0.9533119797706604, 0.9536716938018799, 0.9571115970611572, 0.9626089930534363, 0.95989990234375, 0.9602721929550171, 0.9606471657752991, 0.9610247611999512, 0.964489758014679, 0.9648739099502563, 0.9652606844902039, 0.9656500816345215, 0.966042160987854, 0.9664368629455566, 0.9668341875076294, 0.9641406536102295, 0.96763676404953, 0.9649458527565002, 0.9653524160385132, nan, 0.9657616019248962, 0.9661733508110046, 0.9645201563835144, 0.9649362564086914, 0.9653549194335938, 0.9657762050628662, 0.966200053691864, 0.9666264653205872, 0.9639459848403931, 0.9643761515617371, 0.9648089408874512, 0.9631684422492981, 0.9604902863502502, 0.9609283804893494, 0.961368978023529, 0.9670110940933228, 0.970579981803894, 0.9679097533226013, 0.9683629274368286, 0.9688186049461365, 0.9724035263061523, 0.9697375297546387, 0.9702008366584778, 0.9737977981567383, 0.9711349010467529, 0.9695161581039429, 0.9720789790153503, 0.9725548028945923, 0.9761719703674316, 0.9735139608383179, 0.97190260887146, 0.9723873138427734, 0.9728745222091675, 0.9733642339706421, 0.9738563895225525, 0.9743510484695435, 0.9748481512069702, 0.9753477573394775, 0.9758497476577759, 0.9731979370117188, 0.9768611788749695, 0.97737056016922, 0.9778823256492615, 0.9783965945243835, 0.9789133071899414, 0.9794324040412903, 0.979953944683075, 0.9804779291152954, 0.9810042977333069, 0.978360116481781, 0.9788895845413208, 0.9794214963912964, 0.979955792427063, 0.9783724546432495, 0.9789103269577026, 0.9794506430625916, 0.9799932837486267, 0.9826630353927612, 0.9832116365432739, 0.9837625622749329, 0.9843158721923828, 0.985429584980011, 0.9859899282455444, 0.9897522926330566, 0.9903191924095154, 0.9908884167671204, 0.9861180782318115, 0.9834820032119751, 0.9840533137321472, 0.9846268892288208, 0.9830610156059265, 0.9836379885673523, 0.986361563205719, 0.984798789024353, 0.9907496571540833, 0.9881168603897095, 0.9865571856498718, 0.9892985224723816, 0.9877407550811768, 0.9991021752357483, 0.9997061491012573, 0.9895331859588623, 1.0123242139816284, 1.0129438638687134, 1.0135657787322998, 1.0087839365005493, 1.0148166418075562, 1.0154454708099365, 1.0128268003463745, 1.0080385208129883, 1.0086687803268433, 0.9984484910964966, 0.9914746880531311, 0.992099940776825, 0.9927273988723755, 0.9955331087112427, 0.9961662888526917, 0.9974392652511597, 0.9980789422988892, 0.9987208843231201, 0.9993649125099182, 1.003289818763733, 1.0039403438568115, 1.001310110092163, 1.0019627809524536, 1.0059047937393188, 1.0010814666748047, 1.0017391443252563, 1.0030608177185059, 1.0059236288070679, 1.0043909549713135, 1.0017566680908203, 1.0002217292785645, 1.0008903741836548, 1.0015612840652466, 1.0000265836715698, 0.998059868812561, 0.998735249042511, 0.9994127154350281, 1.0000921487808228, 1.0007736682891846, 0.9992392659187317, 0.9999233484268188, 0.9972777366638184, 0.9979636073112488, 0.9964273571968079, 0.9978059530258179, 0.9962694644927979, 0.9969621896743774, 0.9976569414138794, 0.9983536601066589, 0.9990523457527161, 0.9997530579566956, 1.0011602640151978, 1.0018668174743652, 1.0025752782821655, 1.0055302381515503, 1.0197206735610962, 1.0181986093521118, 1.0189244747161865, 1.0564221143722534, 1.0571796894073486, 1.0545556545257568, 1.0553147792816162, 1.056075930595398, 1.056839108467102, 1.0583714246749878, 1.0591405630111694, 1.0576444864273071, 1.0606848001480103, 1.0591893196105957, 1.059964656829834, 1.061521291732788, 1.0588866472244263, 1.0596673488616943, 1.0604498386383057, 1.0620207786560059, nan]
        # adata = [0.0 for j in range(640)]
        adata[1] = 1.25
        adata[0] = 1.50
        adata[-1] = 1.0
        asus_data = LaserScan()
        asus_data.angle_max = 0.503935337067
        asus_data.angle_min = -0.517280697823
        asus_data.angle_increment = (asus_data.angle_max - asus_data.angle_min)/float(len(adata))
        asus_data.range_max = 10.0
        asus_data.range_min = 0.02
        asus_data.header.seq = seq
        asus_data.header.frame_id = 'camera_depth_frame'
        pub2 = rospy.Publisher('/asus_scan', LaserScan, queue_size=1)
        asus_data.ranges = adata


        fuse_data = LaserScan()
        fuse_data.angle_max = numpy.pi
        fuse_data.angle_min = -numpy.pi
        fuse_data.angle_increment = -numpy.pi/180.0
        fuse_data.range_max = 6.0
        fuse_data.range_min = 0.15
        fuse_data.header.seq = seq
        fuse_data.header.frame_id = 'global_scan'
        pub3 = rospy.Publisher('/scan', LaserScan, queue_size=1)

        import tf
        listener = tf.TransformListener()
        while True:
            try:
                now = rospy.Time.now()
                listener.waitForTransform('camera_depth_frame', 'laser', now, rospy.Duration(1))
                (trans, rot) = listener.lookupTransform('camera_depth_frame', 'laser', now)
                rospy.loginfo('get transform')
                break
            except:
                now = rospy.Time.now()
                listener.waitForTransform('camera_depth_frame', 'laser', now, rospy.Duration(1))

        while not rospy.is_shutdown():
            lidar_data.header.stamp = rospy.Time.now()
            pub1.publish(lidar_data)

            asus_data.header.stamp = rospy.Time.now()
            pub2.publish(asus_data)

            fuse_data.ranges = Data.data_transform(asus_data,lidar_data,trans)
            fuse_data.header.stamp = rospy.Time.now()
            pub3.publish(fuse_data)
            seq += 1
            rate.sleep()

#结束触发
class tester3():
    def __init__(self):
        self.pub = rospy.Publisher('/StopRun_run', Bool, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.pubCB)
        rospy.Subscriber('StopRun_Connected', Bool, self.start)
        rospy.spin()

    def start(self, data):
        pass

    def pubCB(self, event):
        res = raw_input('True / False:')
        if res == 'True':
            res = True
        else:
            res = False
        self.pub.publish(res)

if __name__=='__main__':
     rospy.init_node('Plan_tester_pub')
     try:
         rospy.loginfo( "initialization system")
         tester3()
         rospy.loginfo("process done and quit" )
     except rospy.ROSInterruptException:
         rospy.loginfo("node terminated.")

