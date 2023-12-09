# Example tuples
Xs = (1, 2, 3)
Ys = ('a', 'b', 'c')
Zs = (True, False, True)

commpound_list = []
result = []
# Specify the index i
for i in range(len(Xs)):
    result = []
    result.append(Xs[i])
    result.append(Ys[i])
    result.append(Zs[i])
    commpound_list.append(result)

# Using list comprehension
# result_list = [t[i] for t in (Xs, Ys, Zs)]

# Alternatively, using a loop
# result_list = []
# for t in (Xs, Ys, Zs):
#     result_list.append(t[i])

# print(result_list)
print(commpound_list)