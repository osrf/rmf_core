# This message is published by the fleet adapter in response to a
# DispatchRequest message. It indicates whether the requested task addition or
# cancellation was successful. 

# The DispatchRequest message received by the Fleet Adapter
DispatchRequest dispatch_request

# True if the addition or cancellation operation was successful
bool success
