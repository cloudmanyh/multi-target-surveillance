import graph
import networkx as nx


if __name__ == "__main__":
    target_number, connectivity = 5, 0.6
    G = nx.Graph()
    #拓扑信息、节点信息、边信息
    topology, node_labels = graph.generate_topology(target_number, connectivity)
    graph.build_graph(topology, G)
    edge_labels = nx.get_edge_attributes(G,'weight') # 生成连通边标签
    distance_matrix, path = graph.get_graph_attribute(G)
    print('节点间距离矩阵：\n',distance_matrix)
    print('最短间距路径：\n',path)
    # print('node_labels: ', node_labels)
    # print('edge_labels: ', edge_labels)
    
    graph.draw_graph(G, node_labels, edge_labels)