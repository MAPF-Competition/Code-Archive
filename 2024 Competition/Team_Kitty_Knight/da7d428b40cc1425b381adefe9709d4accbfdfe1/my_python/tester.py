import torch
from torch import nn

class TestPolicy(nn.Module):
    def __init__(self):
        super(TestPolicy, self).__init__()
        self.fc = nn.Linear(4, 1)
        
    def forward(self, x):
        return self.fc(x)

class Tester:
    def __init__(self):
        pass
    
    def test_print(self):
        print("test print: success")
        
    def test_torch(self,v):
        a=torch.tensor([1,2,3], device="cuda")
        print(a,v)
        
    def test_nn(self):
        model = TestPolicy()
        model = model.to("cuda")
        x = torch.zeros([3,4], device="cuda", dtype=torch.float32)
        y = model(x)
        print(y)