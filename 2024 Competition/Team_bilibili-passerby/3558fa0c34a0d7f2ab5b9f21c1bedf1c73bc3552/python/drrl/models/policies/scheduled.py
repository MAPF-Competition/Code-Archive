import math
import time
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions.categorical import Categorical


def layer_init(layer, std=np.sqrt(2), bias_const=0.0):
    torch.nn.init.orthogonal_(layer.weight, std)
    torch.nn.init.constant_(layer.bias, bias_const)
    return layer

class ScaledDotProductAttention(nn.Module):

    def forward(self, query, key, value, mask=None):
        dk = query.size()[-1]
        scores = query.matmul(key.transpose(-2, -1)) / math.sqrt(dk)
        if mask is not None:
            scores = scores.masked_fill(mask == 0, -1e9)
        attention = F.softmax(scores, dim=-1)
        return attention.matmul(value)


class MultiHeadAttention(nn.Module):

    def __init__(self,
                 in_features,
                 head_num,
                 bias=True,
                 activation=F.relu):
        """Multi-head attention.

        :param in_features: Size of each input sample.
        :param head_num: Number of heads.
        :param bias: Whether to use the bias term.
        :param activation: The activation after each linear transformation.
        """
        super(MultiHeadAttention, self).__init__()
        if in_features % head_num != 0:
            raise ValueError('`in_features`({}) should be divisible by `head_num`({})'.format(in_features, head_num))
        self.in_features = in_features
        self.head_num = head_num
        self.activation = activation
        self.bias = bias
        self.linear_q = layer_init(nn.Linear(in_features, in_features, bias))
        self.linear_k = layer_init(nn.Linear(in_features, in_features, bias))
        self.linear_v = layer_init(nn.Linear(in_features, in_features, bias))
        self.linear_o = layer_init(nn.Linear(in_features, in_features, bias))

    def forward(self, q, k, v, mask=None):
        q, k, v = self.linear_q(q), self.linear_k(k), self.linear_v(v)
        if self.activation is not None:
            q = self.activation(q)
            k = self.activation(k)
            v = self.activation(v)

        q = self._reshape_to_batches(q)
        k = self._reshape_to_batches(k)
        v = self._reshape_to_batches(v)
        if mask is not None:
            mask = mask.repeat(self.head_num, 1, 1)
        y = ScaledDotProductAttention()(q, k, v, mask)
        y = self._reshape_from_batches(y)

        y = self.linear_o(y)
        if self.activation is not None:
            y = self.activation(y)
        return y

    @staticmethod
    def gen_history_mask(x):
        """Generate the mask that only uses history data.

        :param x: Input tensor.
        :return: The mask.
        """
        batch_size, seq_len, _ = x.size()
        return torch.tril(torch.ones(seq_len, seq_len)).view(1, seq_len, seq_len).repeat(batch_size, 1, 1)

    def _reshape_to_batches(self, x):
        batch_size, seq_len, in_feature = x.size()
        sub_dim = in_feature // self.head_num
        return x.reshape(batch_size, seq_len, self.head_num, sub_dim)\
                .permute(0, 2, 1, 3)\
                .reshape(batch_size * self.head_num, seq_len, sub_dim)

    def _reshape_from_batches(self, x):
        batch_size, seq_len, in_feature = x.size()
        batch_size //= self.head_num
        out_dim = in_feature * self.head_num
        return x.reshape(batch_size, self.head_num, seq_len, in_feature)\
                .permute(0, 2, 1, 3)\
                .reshape(batch_size, seq_len, out_dim)

    def extra_repr(self):
        return 'in_features={}, head_num={}, bias={}, activation={}'.format(
            self.in_features, self.head_num, self.bias, self.activation,
        )

class ScheduledSampling(nn.Module):
    def __init__(self, cfg):
        super(ScheduledSampling, self).__init__()

        self.cfg = cfg['policy']
        self.placeholder = cfg['policy']['placeholder']

        self.observation_space = self.placeholder['observation_space']
        self.action_space = 21

        n_input_channels = self.observation_space[0]
        self.n_input_channels = n_input_channels

        self._update_count = nn.Parameter(torch.tensor(0), requires_grad=False)
        self._update_time = nn.Parameter(torch.tensor(time.time(), dtype=torch.float64), requires_grad=False)

        self.tasks_hidden = nn.Sequential(
            layer_init(nn.Linear(6, 64)),
            nn.ReLU(),
            layer_init(nn.Linear(64, 256)),
            nn.ReLU(),
            layer_init(nn.Linear(256, 512)),
            nn.ReLU()
        )

        self.tasks_attention = MultiHeadAttention(in_features=512, head_num=2)

        self.share_hidden = nn.Sequential(
            layer_init(nn.Linear(n_input_channels, 512)),
            nn.ReLU(),
            layer_init(nn.Linear(512, 512)),
            nn.ReLU(),
            layer_init(nn.Linear(512, 512)),
            nn.ReLU()
        )

        self.critic = nn.Sequential(
            layer_init(nn.Linear(512, 512)),
            nn.ReLU(),
            layer_init(nn.Linear(512, 512)),
            nn.ReLU(),
            layer_init(nn.Linear(512, 1), std=0.01)
        )

    def forward(self, obs, _x, _y):
        # split obs 0~n_input_channels, n_input_channels
        tasks = obs[:, self.n_input_channels:]
        obs = obs[:, :self.n_input_channels]
        tasks = tasks.reshape(obs.shape[0], 10, 6)

        # hidden
        tasks_hidden = self.tasks_hidden(tasks)
        obs_hidden = self.share_hidden(obs)

        scores = self.tasks_attention(q=tasks_hidden, k=tasks_hidden, v=tasks_hidden)

        # attention_score q: obs, k: scores
        obs_hidden = obs_hidden.reshape(-1, 1, 512)
        attention_scores = torch.multiply(obs_hidden, scores)
        score = torch.mean(attention_scores, dim=-1, keepdim=False) / torch.math.sqrt(512)

        ### value
        value_score = torch.mean(attention_scores, dim=1, keepdim=False) / torch.math.sqrt(512)
        value = self.critic(value_score) # batch_size * n player, 1

        normal_dist = Categorical(logits=score)
        actions = normal_dist.sample()

        log_probs = normal_dist.log_prob(actions)

        return actions, log_probs, value, normal_dist, None

    def action(self, obs, action_mask):
        with torch.no_grad():
            # split obs 0~n_input_channels, n_input_channels
            tasks = obs[:, self.n_input_channels:]
            obs = obs[:, :self.n_input_channels]
            tasks = tasks.reshape(obs.shape[0], 10, 6)

            # hidden
            tasks_hidden = self.tasks_hidden(tasks)
            obs_hidden = self.share_hidden(obs)

            scores = self.tasks_attention(q=tasks_hidden, k=tasks_hidden, v=tasks_hidden)

            # attention_score q: obs, k: scores
            obs_hidden = obs_hidden.reshape(-1, 1, 512)
            attention_scores = torch.multiply(obs_hidden, scores)
            score = torch.mean(attention_scores, dim=-1, keepdim=False) / torch.math.sqrt(512)

            probs = Categorical(logits=score)
            actions = probs.sample()

        return actions, probs

    def batch_step(self, dataset_dict):
        with torch.no_grad():
            batch_size = dataset_dict['obs'].shape[0] # batch_size, n player * obs shape

            actions, log_probs, value, probs, _ = self.forward(dataset_dict['obs'], None, None)
            actions = actions.unsqueeze(-1)
            log_probs = log_probs.unsqueeze(-1)

            results = (actions, log_probs, value)
            results = torch.cat(list(results), dim=-1)

            # Insert update_count to the last column
            update_count = self._update_count
            update_count = update_count * torch.ones((results.shape[0], 1), dtype=torch.float32, device=update_count.device)

            results = torch.cat([results, update_count], dim=-1)

            # batch size, n player, output shape
            results = results.reshape(batch_size, -1)

        return results

    @property
    def update_count(self):
        return self._update_count.item()

    @property
    def update_time(self):
        return self._update_time.item()

    def increment_update(self):
        self._update_count += 1
        self._update_time.data = torch.tensor(time.time(), dtype=torch.float64)
