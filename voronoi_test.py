from scipy.spatial import Voronoi, voronoi_plot_2d
import numpy as np
import random
import math
import heapq
import matplotlib.pyplot as plt
from vor_utils import *
from scipy.interpolate import UnivariateSpline
import scipy.interpolate as interpolate
from smh_astar import *
import pdb

# pose: 
#   pose: 
#     position: 
px= 0.019715886563062668
py= 1.4044371843338013
pz= 0.0
#     orientation: 
qx= 0.0
qy= 0.0
qz= 0.6813265229028491
qw= 0.7319796234794473

yaw = math.atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz))


angle_min = -2.3561899662017822
angle_max = 2.3561899662017822
angle_increment = 0.006554075051099062
time_increment = 0.0
scan_time =0.0
range_min = 0.10000000149011612
range_max = 30.0
ranges = [2.833653211593628, 2.8102400302886963, 2.7975316047668457, 2.7932779788970947, 2.7919719219207764, 2.70410418510437, 2.68740177154541, 2.675257444381714, 2.6715621948242188, 2.671053647994995, 2.5912857055664062, 2.572967290878296, 2.5605239868164062, 2.5561490058898926, 2.5558583736419678, 2.5565383434295654, 2.4706478118896484, 2.456171751022339, 2.4456334114074707, 2.4433343410491943, 2.4405808448791504, 2.4449844360351562, 2.364546537399292, 2.3490943908691406, 2.338441848754883, 2.332746982574463, 2.3308000564575195, 2.332807779312134, 2.294717311859131, 2.2554290294647217, 2.240403175354004, 2.2307941913604736, 2.2276947498321533, 2.2269022464752197, 2.2291207313537598, 2.232057809829712, 2.1638782024383545, 2.1472434997558594, 2.135958433151245, 2.129424571990967, 2.1262922286987305, 2.1279280185699463, 2.13118839263916, 2.1328442096710205, 2.0733258724212646, 2.054784059524536, 2.044229745864868, 2.037532091140747, 2.0339386463165283, 2.033860921859741, 2.033299684524536, 2.0392544269561768, 2.0473079681396484, 1.9819486141204834, 1.9642770290374756, 1.9548625946044922, 1.9508206844329834, 1.9471379518508911, 1.9459036588668823, 1.9480857849121094, 1.9507626295089722, 1.957933783531189, 1.921404242515564, 1.896451711654663, 1.8830734491348267, 1.8748278617858887, 1.8706053495407104, 1.8669605255126953, 1.8650178909301758, 1.8671833276748657, 1.8714711666107178, 1.8793989419937134, 1.890318751335144, 1.8339258432388306, 1.81753408908844, 1.8079780340194702, 1.8032002449035645, 1.8010437488555908, 1.7985330820083618, 1.7971404790878296, 1.7981592416763306, 1.8034511804580688, 1.8107651472091675, 1.8207358121871948, 1.7761313915252686, 1.758666753768921, 1.7477110624313354, 1.7416962385177612, 1.7379180192947388, 1.7354884147644043, 1.7346959114074707, 1.7386057376861572, 1.7416385412216187, 1.745267629623413, 1.7554889917373657, 1.766764521598816, 1.7238438129425049, 1.7090709209442139, 1.7001545429229736, 1.6927711963653564, 1.6881396770477295, 1.686854362487793, 1.6843955516815186, 1.6856738328933716, 1.6889967918395996, 1.696083664894104, 1.7026541233062744, 1.7142794132232666, 1.7045798301696777, 1.6800432205200195, 1.6647138595581055, 1.6575515270233154, 1.6527704000473022, 1.6480804681777954, 1.6479426622390747, 1.645272970199585, 1.650514006614685, 1.6516578197479248, 1.6572072505950928, 1.6671102046966553, 1.6799904108047485, 1.68294358253479, 1.6528706550598145, 1.6417938470840454, 1.6298086643218994, 1.6268326044082642, 1.6219918727874756, 1.621781587600708, 1.6224910020828247, 1.623467206954956, 1.6250407695770264, 1.630216360092163, 1.6366938352584839, 1.6486597061157227, 1.6674963235855103, 1.650615930557251, 1.6327095031738281, 1.6250215768814087, 1.6172370910644531, 1.6113642454147339, 1.6090123653411865, 1.6066789627075195, 1.6075609922409058, 1.611259937286377, 1.6135081052780151, 1.6217283010482788, 1.6305099725723267, 1.6447982788085938, 1.6750332117080688, 1.6429179906845093, 1.6286277770996094, 1.6201707124710083, 1.614128589630127, 1.6100504398345947, 1.6070417165756226, 1.6081361770629883, 1.6101332902908325, 1.6117262840270996, 1.6177419424057007, 1.6221749782562256, 1.633519172668457, 1.6517812013626099, 1.665629267692566, 1.6488713026046753, 1.6389493942260742, 1.6312254667282104, 1.6256616115570068, 1.623674988746643, 1.620688796043396, 1.6218615770339966, 1.6255019903182983, 1.6297047138214111, 1.634148120880127, 1.642520785331726, 1.6563576459884644, 1.703347086906433, 1.6791208982467651, 1.6681369543075562, 1.658970594406128, 1.6533174514770508, 1.649793028831482, 1.6506630182266235, 1.6508731842041016, 1.6515436172485352, 1.6554728746414185, 1.6612061262130737, 1.6680461168289185, 1.6818139553070068, 1.7149417400360107, 1.71562922000885, 1.70271897315979, 1.6960798501968384, 1.6920874118804932, 1.688874363899231, 1.688629388809204, 1.6880587339401245, 1.6926974058151245, 1.6950864791870117, 1.7031952142715454, 1.7123385667800903, 1.7309976816177368, 1.7692893743515015, 1.7552517652511597, 1.7486486434936523, 1.7431710958480835, 1.7399330139160156, 1.7376636266708374, 1.7404451370239258, 1.7416049242019653, 1.7464176416397095, 1.7560958862304688, 1.7645759582519531, 1.7853899002075195, 1.8237488269805908, 1.8142606019973755, 1.8056224584579468, 1.8007451295852661, 1.8016434907913208, 1.8007632493972778, 1.8022791147232056, 1.8064723014831543, 1.8149547576904297, 1.823110818862915, 1.8395100831985474, 1.8921945095062256, 1.8823139667510986, 1.8755422830581665, 1.8716527223587036, 1.8718998432159424, 1.8709291219711304, 1.8757951259613037, 1.883665680885315, 1.8888753652572632, 1.9042917490005493, 1.9707618951797485, 1.9614677429199219, 1.9541972875595093, 1.9527522325515747, 1.9507343769073486, 1.952171802520752, 1.9556748867034912, 1.9615604877471924, 1.9741685390472412, 1.9962223768234253, 2.049724817276001, 2.040891408920288, 2.038487434387207, 2.0400822162628174, 2.0395407676696777, 2.045029401779175, 2.051800012588501, 2.0653624534606934, 2.090926170349121, 2.1372532844543457, 2.1347711086273193, 2.1323049068450928, 2.132887840270996, 2.0672969818115234, 2.047346591949463, 2.033604860305786, 2.0249717235565186, 2.022752285003662, 2.0195472240448, 2.0216877460479736, 2.025294780731201, 2.0326125621795654, 2.0414204597473145, 2.0599606037139893, 2.130709171295166, 2.1258835792541504, 2.126105308532715, 2.1265106201171875, 2.131472110748291, 2.1438915729522705, 2.1553955078125, 2.2396929264068604, 2.2347023487091064, 2.235480546951294, 2.23744797706604, 2.2435457706451416, 2.173191785812378, 2.156707286834717, 2.1474084854125977, 2.138028144836426, 2.139972686767578, 2.1360762119293213, 2.1411378383636475, 2.1496567726135254, 2.159527063369751, 2.177356719970703, 2.2572858333587646, 2.2587575912475586, 2.260721206665039, 2.2677972316741943, 2.2187259197235107, 2.1960597038269043, 2.1848971843719482, 2.175194501876831, 2.171643018722534, 2.1705470085144043, 2.1753482818603516, 2.1802361011505127, 2.1894731521606445, 2.204535484313965, 2.9915833473205566, 2.9771318435668945, 2.970019578933716, 2.9689390659332275, 2.970386266708374, 2.9820683002471924, 3.0039312839508057, 3.0996408462524414, 3.105375051498413, 3.1205687522888184, 3.169881820678711, 3.57025146484375, 3.5836174488067627, 3.61892032623291, 3.706587076187134, 3.723665714263916, 3.6597390174865723, 3.6437594890594482, 3.634791374206543, 3.63632869720459, 3.6495654582977295, 3.673410415649414, 3.7783477306365967, 3.79733943939209, 3.7409725189208984, 3.7218267917633057, 3.7151405811309814, 3.7173728942871094, 3.727013349533081, 3.215860605239868, 3.197021722793579, 3.1910767555236816, 3.1886820793151855, 3.195483446121216, 3.2080416679382324, 3.1961605548858643, 3.162912130355835, 3.1498591899871826, 3.141174793243408, 3.14274001121521, 3.1498725414276123, 3.1656036376953125, 3.788548231124878, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, 2.300710439682007, 2.281278371810913, 2.268322706222534, 2.2645905017852783, 2.261089324951172, 2.2609150409698486, 2.266017436981201, 2.2712349891662598, 2.2864527702331543, 2.311469793319702, 2.4503073692321777, 2.425405979156494, 2.414384603500366, 2.4072470664978027, 2.4041218757629395, 2.4036402702331543, 2.409986734390259, 2.4185760021209717, 2.4342939853668213, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf, 2.1730363368988037, 2.160623073577881, 2.1514229774475098, 2.1454319953918457, 2.144127130508423, 2.1453685760498047, 2.1471991539001465, 2.1562092304229736, 2.1671481132507324, 2.191033124923706, 2.2082746028900146, 2.1921534538269043, 2.075418710708618, 2.05385684967041, 2.0429885387420654, 2.0362043380737305, 2.0329720973968506, 2.030301094055176, 2.0342116355895996, 2.0357563495635986, 2.0462963581085205, 2.0562777519226074, 2.0801100730895996, math.inf, math.inf, 3.881694793701172, 3.848001718521118, 3.8352952003479004, 3.832970142364502, 3.8401012420654297, 3.859166383743286, 3.911970376968384, 3.892651081085205, 3.778515338897705, 3.755849838256836, 3.748872995376587, 3.7474639415740967, 3.757843017578125, 3.7845969200134277, 4.6172895431518555, 4.61716890335083, 4.627771854400635, 4.7082061767578125, 4.680539608001709, 4.679752349853516, 4.686311721801758, 4.715748310089111, math.inf, math.inf, math.inf, math.inf, math.inf, 1.6492948532104492, 1.6272045373916626, 1.6150246858596802, 1.605814814567566, 1.6015167236328125, 1.5985260009765625, 1.5979845523834229, 1.5967415571212769, 1.5978941917419434, 1.6013448238372803, 1.6065623760223389, 1.6133501529693604, 1.6255913972854614, 1.6465938091278076, 1.6948230266571045, 1.6845927238464355, 1.6827524900436401, 1.678379774093628, 1.6811169385910034, 1.680545687675476, 1.6842527389526367, 1.6912686824798584, 1.6967545747756958, 1.7099403142929077, 1.7322503328323364, 3.4846174716949463, 3.480098009109497, 3.417546033859253, 3.381342887878418, 3.36796498298645, 3.365720748901367, 3.36967396736145, 3.375126600265503, 3.400271415710449, 3.635354995727539, 3.8927338123321533, 3.8882548809051514, 3.89272141456604, 3.7982399463653564, 3.7825615406036377, 3.7780442237854004, 3.7815768718719482, 3.6931350231170654, 3.678046464920044, 3.670241117477417, 3.6722586154937744, 3.6037092208862305, 3.579758405685425, 3.5698487758636475, 3.567348003387451, 3.5731306076049805, 3.4931628704071045, 3.47560715675354, 3.467642307281494, 3.468480348587036, 3.476083278656006, 3.3926339149475098, 3.3770699501037598, 3.3702003955841064, 3.370252847671509, 3.380563497543335, 3.3058111667633057, 3.288043260574341, 3.2776808738708496, 3.278104066848755, 3.2831647396087646, 3.236571788787842, 3.207010507583618, 3.1931557655334473, 3.190716505050659, 3.1885857582092285, 3.199399471282959, 3.144632577896118, 3.1201095581054688, 2.93904447555542, 2.9132254123687744, 2.901775360107422, 2.897336006164551, 2.8976263999938965, 2.904232978820801, 2.914888620376587, 2.9423563480377197, 3.0282185077667236, 3.03033185005188, 3.044914722442627, 2.977412223815918, 2.961834192276001, 2.953763484954834, 2.950812339782715, 2.9550867080688477, 2.9647328853607178, 2.937152147293091, 2.905038833618164, 2.8944075107574463, 2.883089780807495, 2.7213149070739746, 2.7006490230560303, 2.688166618347168, 2.6860265731811523, 2.531778335571289, 2.5084636211395264, 2.4946582317352295, 2.4897358417510986, 2.4870619773864746, 2.4885904788970947, 2.49525785446167, 2.000229835510254, 1.9807512760162354, 1.9708971977233887, 1.9626517295837402, 1.9597961902618408, 1.9597880840301514, 1.9612332582473755, 1.8116645812988281, 1.7926454544067383, 1.7844725847244263, 1.7775315046310425, 1.7732508182525635, 1.770753264427185, 1.771605134010315, 1.7721346616744995, 1.7774310111999512, 1.7833751440048218, 1.7923672199249268, 1.809653401374817, 1.927256464958191, 1.9378092288970947, 1.9554928541183472, 2.7034456729888916, 2.6976609230041504, 2.66729736328125, 2.6537926197052, 2.64851975440979, 2.6446235179901123, 2.6478116512298584, 2.6532928943634033, 2.517334461212158, 2.4924545288085938, 2.4822771549224854, 2.473592519760132, 2.472825765609741, 2.471611738204956, 2.4786109924316406, 2.4908719062805176, 2.5071921348571777, 2.655949115753174, 2.644991159439087, 2.6222567558288574, 2.607928991317749, 2.6032652854919434, 2.602205753326416, 2.6060640811920166, 2.614025354385376, 2.62492036819458, 2.659888744354248, 2.6219873428344727, 2.604532241821289, 2.598090410232544, 2.594794273376465, 2.5956015586853027, 2.6015334129333496, 2.6102027893066406, 2.627875328063965, 2.638265609741211, 2.6133501529693604, 2.602632761001587, 2.596745729446411, 2.594899892807007, 2.595935821533203, 2.601940870285034, 2.6163012981414795, 2.643876075744629, 2.634415626525879, 2.6179537773132324, 2.6066043376922607, 2.6028103828430176, 2.6031720638275146, 2.6065175533294678, 2.6152212619781494, 2.633314371109009, 2.6668777465820312, 2.64060378074646, 2.627607822418213, 2.623450517654419, 2.618847131729126, 2.6199402809143066, 2.6286814212799072, 2.6384525299072266, 2.6663331985473633, 2.6733479499816895, 2.655648946762085, 2.6466622352600098, 2.643839120864868, 2.6438708305358887, 2.6493191719055176, 2.6608800888061523, 2.681427478790283, 2.7099313735961914, 2.689723491668701, 2.681018352508545, 2.6768152713775635, 2.677398681640625, 2.680544376373291, 2.68983793258667, 2.7106099128723145, 2.75142240524292, 2.7321221828460693, 2.721682071685791, 2.7176711559295654, 2.716799736022949, 2.721808910369873, 2.732667922973633, 2.7530415058135986, 2.793123960494995, 2.775800943374634, 2.7680375576019287, 2.7624170780181885, 2.765941858291626, 2.7729177474975586, 2.784947156906128, 2.8145105838775635, 2.8376455307006836, 2.826382875442505, 2.818889856338501, 2.8199901580810547, 2.8254799842834473, 2.8346333503723145, 2.855402946472168, 2.9005234241485596, 2.8879289627075195, 2.8808095455169678, 2.880260467529297, 2.8848421573638916, 2.8949484825134277, 2.915836811065674, 2.9671971797943115, 2.953725814819336, 2.948855400085449, 2.948495864868164, 2.953761577606201, 2.9672818183898926, 2.989670515060425, 3.0324690341949463, 3.022706985473633, 3.0199496746063232, 3.0230166912078857, 3.032059907913208, 3.050287961959839, 3.1178109645843506, 3.1059274673461914, 3.1004016399383545, 3.100194215774536, 3.1076927185058594, 3.1239206790924072, 3.202735185623169, 3.1887691020965576]


# points = []

# for i in range (0, 100):
#     point = [random.uniform(0, 10), random.uniform(0, 10)]
#     points.append(point)

points = ranges_to_coordinates(ranges, angle_min + yaw, angle_increment)


for i in range(len(points)):
    x, y = points[i]
    x += -0.055  # hardcoded laser -> base_link
    x = round(x  * 10 / 2) *2   # scale by 10, discretize by 2, make it an int
    y = round(y * 10 /2) *2
    points[i] = (x, y)

points1 = [(0,100), (0,0), (-50,-10),(-40,-10), (50,-10)]
vor = Voronoi(points1, incremental = True)
vor.add_points(points)
# points2 = [(x*2, -10) for x in range (-15, 15)]
# points2 = generate_arc_points(-math.pi/2, 0, 10, 10)
# points2 = generate_ellipse_arc(20, 15, -math.pi, 0, 20)
# vor1 = vor
# vor.add_points(points2)
# points1+=points
# points = points1 + points2

fig = voronoi_plot_2d(vor)
# plt.show()




start = (0, 0)
goal = (100*math.cos(yaw), 100* math.sin(yaw))
pdb.set_trace()
map = get_edge_map2(vor, (0,0))
for p in vor.regions[vor.point_region[0]]:
    print(p, vor.vertices[p][0], vor.vertices[p][1])

# print(map)
# for e in edges:
#     print(vertices[e[0]], vertices[e[1]])


rx = [p[0] for p in points]
ry = [p[1] for p in points]
plt.plot(rx, ry, 'ko')
pdb.set_trace()
vor_vertices = np.append(vor.vertices, [[0,0]], axis=0)
vor_vertices = np.append(vor_vertices, [[0,100]], axis=0)
old_path = a_star(len(vor.vertices), len(vor.vertices)+1, map, vor_vertices, None)
path = a_star(len(vor.vertices), len(vor.vertices)+1, map, vor_vertices, old_path)

print(path)
x = [p[0] for p in path]
y = [p[1] for p in path]
plt.plot(x, y, 'bo')

# tck, u = interpolate.splprep([x, y])
# x_i, y_i = interpolate.splev(np.linspace(0, 1, 100), tck)

# plt.plot(x_i, y_i, 'r')


plt.plot([start[0], goal[0]], [start[1], goal[1]], 'go')
plt.axis([-50, 50, -30, 110])
# plt.savefig("test2.png")
plt.show()