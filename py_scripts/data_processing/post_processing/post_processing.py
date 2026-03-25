import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

state_pred = np.loadtxt("/Users/moaadil/audi_a2dc/state_estimation_copy/src/test/X_upd.csv", delimiter=",")

# with pandas — easier to inspect
df = pd.read_csv("/Users/moaadil/audi_a2dc/state_estimation_copy/src/test/X_upd.csv",
                 header=None,
                 names=["x", "y", "psi"])
print(df.head())
