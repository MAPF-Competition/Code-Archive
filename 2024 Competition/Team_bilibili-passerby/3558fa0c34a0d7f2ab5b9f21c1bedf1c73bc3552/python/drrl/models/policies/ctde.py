import time

import torch
import torch.nn as nn
from torch.distributions.categorical import Categorical

import numpy as np


def layer_init(layer, std=np.sqrt(2), bias_const=0.0):
    torch.nn.init.orthogonal_(layer.weight, std)
    torch.nn.init.constant_(layer.bias, bias_const)
    return layer

def batch_to_seq(tensor_batch, n_steps):
    # seq length, mini batch size, layer size
    layer_size = tensor_batch.shape[-1]
    mini_batch_size = tensor_batch.shape[0] // n_steps
    tensor_batch = torch.reshape(tensor_batch, [mini_batch_size, n_steps, layer_size])
    return [torch.squeeze(v, [1]) for v in torch.split(tensor_batch, [1] * n_steps, dim=1)]

def seq_to_batch(tensor_seq):
    number_of_hidden = tensor_seq[0].shape[-1]
    return torch.cat(tensor_seq, dim=1).reshape(-1, number_of_hidden)

class CTDETeamPolicy(nn.Module):

    def __init__(self, cfg, n_lstm_steps=1) -> None:
        super(CTDETeamPolicy, self).__init__()

        self.cfg = cfg['policy']
        self.placeholder = cfg['policy']['placeholder']

        # input information
        self.observation_space = self.placeholder['observation_space']
        self.action_space = self.placeholder['action_space']
        self.n_lstm_hidden_size = cfg['n_lstm']
        self.n_lstm_steps = n_lstm_steps

        self._update_count = nn.Parameter(torch.tensor(0), requires_grad=False)
        self._update_time = nn.Parameter(torch.tensor(time.time(), dtype=torch.float64), requires_grad=False)

        n_input_channels = self.observation_space[0]
        self.n_input_channels = n_input_channels

        self.share_hidden = nn.Sequential(
            layer_init(nn.Linear(n_input_channels, 512)),
            nn.ReLU(),
            layer_init(nn.Linear(512, 512)),
            nn.ReLU(),
            layer_init(nn.Linear(512, 512)),
            nn.ReLU()
        )

        self.lstm = nn.LSTM(512 + 32, self.n_lstm_hidden_size, batch_first=True)
        for name, param in self.lstm.named_parameters():
            if "bias" in name:
                nn.init.constant_(param, 0)
            elif "weight" in name:
                nn.init.orthogonal_(param, 1.0)

        self.critic = nn.Sequential(
            layer_init(nn.Linear(512, 512)),
            nn.ReLU(),
            layer_init(nn.Linear(512, 512)),
            nn.ReLU(),
            layer_init(nn.Linear(512, 1), std=0.01)
        )

        self.actor = nn.Sequential(
            layer_init(nn.Linear(512, 512)),
            nn.ReLU(),
            layer_init(nn.Linear(512, 512)),
            nn.ReLU(),
            layer_init(nn.Linear(512, self.action_space), std=0.01)
        )

        self.pre_action = nn.Sequential(
            nn.Embedding(4, 32),
            layer_init(nn.Linear(32, 32)),
            nn.ReLU(),
            layer_init(nn.Linear(32, 32)),
            nn.ReLU(),
        )

    @property
    def update_count(self):
        return self._update_count.item()

    @property
    def update_time(self):
        return self._update_time.item()


    def forward(self, obs, mask, lstm_state):
        ### share memory
        obs = obs.reshape(-1, self.n_input_channels) # Batch size, obs shape
        pre_action = obs[:, 0]
        obs_hidden = self.share_hidden(obs) # Batch size, obs shape
        pre_action_hidden = self.pre_action(pre_action.long())
        share_hidden = torch.cat((obs_hidden, pre_action_hidden), dim=-1)
        ### LSTM
        seq_lstm_share_memory = batch_to_seq(share_hidden, self.n_lstm_steps) #  Seq Length, Batch Size // Seq Length, obs shape
        if self.n_lstm_steps != 1:
            lstm_state = lstm_state.reshape(obs.shape[0] // self.n_lstm_steps, self.n_lstm_steps, -1)[:, 0]
        seq_lstm_share_memory_hidden, lstm_state = self.lstm_process(seq_lstm_share_memory, lstm_state)
        lstm_share_memory_hidden = seq_to_batch(seq_lstm_share_memory_hidden)

        logits = lstm_share_memory_hidden
        ### value
        value = self.critic(logits) # batch_size * n player, 1

        ### actor
        mask = mask.reshape(-1, 4)
        logits = self.actor(logits) # batch_size * n player, action space
        logits = logits + (1 - mask) * -999999.0 # action mask is bool, replace the invalid action with -999999
        normal_dist = Categorical(logits=logits)
        actions = normal_dist.sample()
        log_probs = normal_dist.log_prob(actions)
        actions = actions.reshape(-1, 1)
        log_probs = log_probs.reshape(-1, 1)

        return actions, log_probs, value, normal_dist, lstm_state

    def batch_step(self, dataset_dict):
        with torch.no_grad():
            batch_size = dataset_dict['obs'].shape[0] # batch_size, n player * obs shape

            actions, log_probs, value, probs, lstm_state = self.forward(dataset_dict['obs'], dataset_dict['mask'], dataset_dict['lstm_state'])
            results = (actions, log_probs, value, lstm_state)
            results = torch.cat(list(results), dim=-1)

            # Insert update_count to the last column
            update_count = self._update_count
            update_count = update_count * torch.ones((results.shape[0], 1), dtype=torch.float32, device=update_count.device)

            results = torch.cat([results, update_count], dim=-1)

            # batch size, n player, output shape
            results = results.reshape(batch_size, -1)

        return results

    def action(self, obs, action_mask, lstm_state):
        with torch.no_grad():
            pre_action = obs[:, 0]
            obs_hidden = self.share_hidden(obs)
            pre_action_hidden = self.pre_action(pre_action.long())
            share_hidden = torch.cat((obs_hidden, pre_action_hidden), dim=-1)

            seq_lstm_share_memory = batch_to_seq(share_hidden, self.n_lstm_steps) #  Seq Length, Batch Size // Seq Length, obs shape
            seq_lstm_share_memory_hidden, lstm_state = self.lstm_process(seq_lstm_share_memory, lstm_state)
            lstm_share_memory_hidden = seq_to_batch(seq_lstm_share_memory_hidden)
            logits = self.actor(lstm_share_memory_hidden)
            # action mask is bool, replace the invalid action with -999999
            logits = logits + (1 - action_mask) * -999999.0
            probs = Categorical(logits=logits)
            # actions = probs.mode # probs.sample()
            actions = probs.sample()

        return actions, lstm_state

    def lstm_process(self, x, hidden_state):
        layer_size = hidden_state.shape[-1] // 2
        hn, cn = torch.split(hidden_state, [layer_size, layer_size], dim=1)
        hn = hn.reshape(1, -1, layer_size)
        cn = cn.reshape(1, -1, layer_size)

        # torch LSTM batch_first only for input, output, but not for h_0, c_0
        new_hidden = []
        for idx, h in enumerate(x):
            h, (hn, cn) = self.lstm(h.unsqueeze(1), (hn.contiguous(), cn.contiguous()))
            new_hidden += [h]
        lstm_state = torch.cat((hn.squeeze(0), cn.squeeze(0)), dim=1) # (batch, hidden_szie*2)
        return new_hidden, lstm_state

    def increment_update(self):
        self._update_count += 1
        self._update_time.data = torch.tensor(time.time(), dtype=torch.float64)
