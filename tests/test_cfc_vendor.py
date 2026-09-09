import unittest

import torch

from ca_lsc_td3._vendor.torch_cfc import Cfc


class VendoredCfcTests(unittest.TestCase):
    @staticmethod
    def _model() -> Cfc:
        return Cfc(
            12,
            16,
            1,
            {
                "backbone_activation": "silu",
                "backbone_units": 24,
                "backbone_layers": 1,
            },
        )

    def test_timespans_change_output_and_gradients_are_finite(self):
        torch.manual_seed(7)
        model = self._model()
        sequence = torch.randn((2, 8, 12), dtype=torch.float32)
        short_dt = torch.full((2, 8), 0.05, dtype=torch.float32)
        long_dt = torch.full((2, 8), 0.20, dtype=torch.float32)

        short_output = model(sequence, short_dt)
        long_output = model(sequence, long_dt)
        self.assertEqual(short_output.shape, (2, 1))
        self.assertTrue(torch.isfinite(short_output).all().item())
        self.assertFalse(torch.allclose(short_output, long_output))

        short_output.square().mean().backward()
        gradients = [
            parameter.grad
            for parameter in model.parameters()
            if parameter.grad is not None
        ]
        self.assertTrue(gradients)
        self.assertTrue(
            all(torch.isfinite(gradient).all().item() for gradient in gradients)
        )


if __name__ == "__main__":
    unittest.main()

