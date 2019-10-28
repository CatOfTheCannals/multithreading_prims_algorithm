import networkx as nx
from networkx.generators.classic import star_graph
import random

for i in range(100,1001,100):
    n = i
    
    G = nx.star_graph(n-1) # estrella
    for (u, v, w) in G.edges(data=True):
        w['weight'] = random.randint(1, n)

    m = G.number_of_edges()
    tamanios = "{}\n{}\n".format(n, m)
    tamaniosBytes = bytes(tamanios, "utf-8")
    with open("experimentacion/estrella/estrella" + str(i) + ".txt", "wb") as file:  # te hace el open y el close
        file.write(tamaniosBytes)
        nx.write_weighted_edgelist(G, file)

