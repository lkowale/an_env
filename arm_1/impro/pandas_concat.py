import pandas as pd

df1 = pd.DataFrame({'A': ['A0'],
                     'B': ['B0'],
                     'C': ['C0'],
                     'D': ['D0']})

df2 = pd.DataFrame({'A': ['A4'],
                     'B': ['B4'],
                     'C': ['C4'],
                     'D': ['D4']})

print(df1.to_string())
print(df2.to_string())

df3= pd.concat([df1, df2], axis=1)
print(df3.to_string())

print(df3.iloc[0]['A'])

for param in list(df3.columns.values)\
        :
    print(f' {param}')

