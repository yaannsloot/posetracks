from . import apply
from . import datatypes
from . import utils
from .datatypes import Image

def prepare_context_rgb(input):
    if isinstance(input, Image):
        input = [input]
    return {
        "input_sizes": [i.size for i in input]
    }

_CONTEXT_FUNCS = {
    "image": prepare_context_rgb
}


def infer(model, input):
    model = utils.MODEL_DEFS[model]
    context = _CONTEXT_FUNCS[model["input_type"]](input)

    for call in model["preprocess"]:
        input = apply(input, call["op"], context=context, **call)

    if not isinstance(input, datatypes.Tensor):
        raise RuntimeError(
            f"Error executing {model["name"]}. Preprocessing did not produce a tensor.")

    in_label = model.get("metadata", {}).get("in", "input")
    out_labels = model.get("metadata", {}).get("out", "output")
    
    if not isinstance(out_labels, list):
        out_labels = [out_labels]    

    input = {in_label: input}

    session = utils.prepare_session(model["name"], model["path"])

    # Also might need to be changed after further tests
    output = session.run(out_labels, input)
    output = {out_labels[i]: output[i] for i in range(len(output))}

    for call in model["postprocess"]:
        output = apply(output, call["op"], context=context, **call)

    return output

