import random
import numpy as np  # 使用import导入模块numpy，并简写成np
from paraTags import paraTags
import networkx as nx
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif']=['SimHei'] #用来正常显示中文标签
plt.rcParams['axes.unicode_minus']=False #用来正常显示负号

# 输入：节点数量和节点间连接度
# 功能：根据目标点数量与节点点连接度，生成目标点之间的加权无向拓扑结构图
# 输出：返回网络拓扑信息列表和节点标签信息
def generate_topology(target_number, connectivity):
    topology = []
    labels = {}
    node_id_list = []
    for i in range(target_number):
        for j in range(i+1, target_number):
            link_flag = random.random() # 随机生成是否连接的标志值
            if link_flag <= connectivity: # 当连接标志值小于连接度的时候，表示两边相连
                if i not in node_id_list:
                    node_id_list.append(i)
                if j not in node_id_list:
                    node_id_list.append(j)
                w = round(random.uniform(paraTags.distance_lower,paraTags.distance_upper),1)
                topology.append([i, j, w])
    node_id_list.sort()
    for k in node_id_list:
        labels[k] = str(k)
    return topology, labels

# 输入：空白图结构和网络拓扑信息列表
# 功能：根据目标点数量与目标点连接度，生成目标点之间的加权无向拓扑结构图
# 输出：无返回值
def build_graph(topology, graph):
    for t in topology:
        graph.add_edge(t[0], t[1], weight=t[2])

# 输入：元素为元组的列表，元组第一位为节点id
# 功能：根据节点id从小到大的顺序，对列表重新排序
# 输出：返回排序后的列表
def list_sort_by_tuple_id(L):
    sorted_list = []
    for i in range(len(L)):
        for l in L:
            if l[0] == i:
                sorted_list.append(l)
                break
    return sorted_list

# 输入：词典
# 功能：将词典中所有元素的value值合为一个list
# 输出：合成后的list
def dict_value_to_list(D):
    L = []
    for i in D.keys():
        L.append(D[i])
    return L

# 输入：构造后的图结构
# 功能：根据构造后的图结构，获得图的基本属性
# 输出：返回图中任意两点之间的最短路径和最短路径遍历途径
def get_graph_attribute(graph):
    path=list(nx.all_pairs_dijkstra_path(graph)) # 生成每对节点之间的最短路径
    path_list = list_sort_by_tuple_id(path) # 最短路径列表按照元组id从小到大排序
    update_path_list = []
    for p in path_list:
        path_dict = p[1]
        data = dict(sorted(path_dict.items(), key=lambda x: x[0])) # 词典按照key值大小排序 
        update_path_list.append((p[0],data))
    length=list(nx.all_pairs_dijkstra_path_length(graph)) # 生成每对节点之间的最短距离
    length_list = list_sort_by_tuple_id(length) # 最短距离列表按照元组id从小到大排序
    dis_list = []
    for l in length_list:
        temp_list = []
        len_dict = l[1]
        data = dict(sorted(len_dict.items(), key=lambda x: x[0])) # 词典按照key值大小排序
        temp_list = dict_value_to_list(data)
        dis_list.append(temp_list)
    return dis_list, update_path_list

# 输入：构造后的图结构，节点标签，边标签
# 功能：根据构造后的图结构，节点标签，边标签，可视化形式绘制图结构
# 输出：无返回值
def draw_graph(graph, node_labels, edge_labels):
    pos=nx.spring_layout(graph) # 生成节点位置 
    nx.draw_networkx_nodes(graph,pos,node_color='#FFE4C4',node_size=300,alpha=1) # 把节点画出来
    nx.draw_networkx_edges(graph,pos,width=1.0,alpha=1,edge_color='#800080') # 把边画出来 
    nx.draw_networkx_labels(graph,pos,node_labels,font_size=16, font_color='#000000') # 把节点的标签画出来
    nx.draw_networkx_edge_labels(graph, pos, edge_labels) # 把边权重画出来 
    #显示graph
    plt.title('有权图')
    plt.axis('on')
    plt.xticks([])
    plt.yticks([])
    plt.show()



if __name__ == "__main__":
    target_number, connectivity = 5, 0.6
    G = nx.Graph()
    #拓扑信息、节点信息、边信息
    topology, node_labels = generate_topology(target_number, connectivity)
    build_graph(topology, G)
    edge_labels = nx.get_edge_attributes(G,'weight') # 生成连通边标签
    distance_matrix, path = get_graph_attribute(G)
    print('节点间距离矩阵：\n',distance_matrix)
    print('最短间距路径：\n',path)
    draw_graph(G, node_labels, edge_labels)
