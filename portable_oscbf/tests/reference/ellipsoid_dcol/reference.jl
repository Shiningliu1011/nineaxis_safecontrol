using DelimitedFiles
using LinearAlgebra
using StaticArrays
import DifferentiableCollisions as DCOL

inputs = readdlm(ARGS[1], ',')
results = zeros(size(inputs, 1))
for i in axes(inputs, 1)
    ca, cb = SVector{3}(inputs[i, 1:3]), SVector{3}(inputs[i, 13:15])
    la, lb = reshape(inputs[i, 4:12], 3, 3), reshape(inputs[i, 16:24], 3, 3)
    pa, pb = inv(la * la'), inv(lb * lb')
    a = DCOL.Ellipsoid(SMatrix{3,3}((pa + pa') / 2))
    b = DCOL.Ellipsoid(SMatrix{3,3}((pb + pb') / 2))
    a.r, b.r = ca, cb
    scale, _ = DCOL.proximity(a, b; pdip_tol=1e-12)
    results[i] = scale
end
writedlm(ARGS[2], results, ',')
