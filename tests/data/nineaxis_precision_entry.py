import json

import jax
from work.nineaxis_model import NineaxisModel

jax.config.update("jax_enable_x64", False)
model = NineaxisModel.load()
assert jax.config.x64_enabled
assert str(model.jax_parameters.screw_axes.dtype) == "float64"
assert str(model.jax_parameters.tcp_zero.dtype) == "float64"
model.fk((model.position_lower + model.position_upper) / 2)
print(json.dumps({"x64_enabled": jax.config.x64_enabled, "dtype": str(model.jax_parameters.screw_axes.dtype)}))
