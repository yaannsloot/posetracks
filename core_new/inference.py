from . import apply
from . import datatypes
from . import utils
from .datatypes import Tensor, Image, BoundingBox, Context


def img_metadata(input):
    if isinstance(input, Image):
        input = [input]
    return {
        "input_sizes": [i.size for i in input]
    }


_METADATA_FUNCS = {
    "image": img_metadata
}


def infer(model, input):
    model = utils.MODEL_DEFS[model]
    metadata = _METADATA_FUNCS[model["input_type"]](input)

    context = Context(input, metadata=metadata)

    # Get the string or list of strings for model input
    # This is the context value expected after preprocessing
    ctx_in = model["model_in"]

    for i, call in enumerate(model["preprocess"]):
        try:
            apply(call["op"], context, **call)
        except Exception as e:
            raise RuntimeError(
                f'Failed to execute {i}:{call['op']} during preprocessing') from e

    # Prepare inputs for model
    if not isinstance(ctx_in, list):
        model_in = [ctx_in]
        try:
            context.expect(ctx_in)
            ctx_in = model_in
        except:
            # Expected a named value from preprocessing but no such name exists
            # In this case, it is assumed that the default output is the intended input
            ctx_in = ["value"]
    else:
        model_in = ctx_in

    # Sanity check
    context.expect_type(**{i: Tensor for i in ctx_in})

    # Final input for inference
    values = {k: context[v] for k, v in zip(model_in, ctx_in)}

    # Names of model outputs in order
    names_out = model["model_out"]
    if not isinstance(names_out, list):
        names_out = [names_out]

    # Execute
    session = utils.prepare_session(model["name"], model["path"])
    predictions = session.run(names_out, values)

    # Update context with model predictions
    for k, v in zip(names_out, predictions):
        context[k] = v
    context.update(predictions[0] if len(predictions) == 1 else predictions)

    # Postprocess
    for i, call in enumerate(model["postprocess"]):
        try:
            apply(call["op"], context, **call)
        except Exception as e:
            raise RuntimeError(
                f'Failed to execute {i}:{call['op']} during postprocessing') from e

    # Get expected postprocessing outputs
    final_names = model["output_type"]
    if not isinstance(final_names, list):
        output_type = [final_names]
        try:
            context.expect(final_names)
            final_names = output_type
        except:
            final_names = ["value"]
    else:
        output_type = final_names
        context.expect(*final_names)
        
    return {k: context[v] for k, v in zip(output_type, final_names)}
