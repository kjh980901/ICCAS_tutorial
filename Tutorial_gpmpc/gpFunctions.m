function varargout = gpFunctions(functionName, varargin)
%% Gaussian Process Functions
% Collection of GP-related functions for GP-MPC Tutorial
%
% Usage:
%   [norm_data, mu, sigma] = gpFunctions('normalize', data, mu, sigma)
%   subset_data = gpFunctions('selectSubsetOfData', original_data, num_points)
%   gp_models = gpFunctions('trainGPModels', train_data, I)
%   hyp_opt = gpFunctions('optimizeHyperparameters', X, Y, hyp_init)
%   nlml = gpFunctions('negativeLogMarginalLikelihood', hyp, X, Y)
%   K = gpFunctions('seKernel', X1, X2, hyp)
%   [mu, sigma2] = gpFunctions('gpPredict', model, X_test_norm)

switch functionName
    case 'normalize'
        [varargout{1}, varargout{2}, varargout{3}] = normalize(varargin{:});
    case 'selectSubsetOfData'
        varargout{1} = selectSubsetOfData(varargin{:});
    case 'trainGPModels'
        varargout{1} = trainGPModels(varargin{:});
    case 'optimizeHyperparameters'
        varargout{1} = optimizeHyperparameters(varargin{:});
    case 'negativeLogMarginalLikelihood'
        varargout{1} = negativeLogMarginalLikelihood(varargin{:});
    case 'seKernel'
        varargout{1} = seKernel(varargin{:});
    case 'gpPredict'
        [varargout{1}, varargout{2}] = gpPredict(varargin{:});
    otherwise
        error('Unknown function name: %s', functionName);
end

end

%% [Improvement] Data subset selection function using K-means
function subset_data = selectSubsetOfData(original_data, num_points)
    if size(original_data.X, 1) <= num_points
        subset_data = original_data;
        return;
    end
    
    % Perform K-means clustering (based on input space)
    % kmeans returns centroids of each cluster.
    % These centroids represent the distribution of the data.
    [~, centroids] = kmeans(original_data.X_norm, num_points, 'MaxIter', 200);
    
    % Find the closest actual data point to each centroid.
    % 'knnsearch' finds the index of the nearest neighbor.
    selected_indices = knnsearch(original_data.X_norm, centroids);
    
    % Remove duplicate indices
    selected_indices = unique(selected_indices);
    
    % Create subset using selected indices
    subset_data = original_data; % Copy structure
    subset_data.X = original_data.X(selected_indices, :);
    subset_data.Y = original_data.Y(selected_indices, :);
    subset_data.X_norm = original_data.X_norm(selected_indices, :);
end

function [norm_data, mu, sigma] = normalize(data, mu, sigma)
    if nargin < 2
        mu = mean(data, 1);
        sigma = std(data, 1);
    end
    sigma(sigma < 1e-9) = 1; 
    norm_data = (data - mu) ./ sigma;
end

function gp_models = trainGPModels(train_data, I)
    n_outputs = length(I.states);
    gp_models = cell(n_outputs, 1);

    for i = 1:n_outputs
        fprintf('  - Training GP model for state %d error...\n', i);
        D = size(train_data.X_norm, 2);
        hyp_init = [zeros(D, 1); log(std(train_data.Y(:, i))); log(0.1)];
        
        hyp_opt = optimizeHyperparameters(train_data.X_norm, train_data.Y(:, i), hyp_init);
        
        model.hyp = hyp_opt;
        model.X_train = train_data.X_norm;
        model.Y_train = train_data.Y(:, i);
        model.X_mean = train_data.X_mean;
        model.X_std = train_data.X_std;
        gp_models{i} = model;
        
        fprintf('    -> Optimization complete. Signal_std=%.3f, Noise_std=%.3f\n', exp(hyp_opt(D+1)), exp(hyp_opt(D+2)));
    end
end

function hyp_opt = optimizeHyperparameters(X, Y, hyp_init)
    obj_fun = @(hyp) negativeLogMarginalLikelihood(hyp, X, Y);
    options = optimoptions('fminunc', 'Algorithm','quasi-newton','Display','none','MaxIterations',200,'OptimalityTolerance',1e-6);
    hyp_opt = fminunc(obj_fun, hyp_init, options);
end

function nlml = negativeLogMarginalLikelihood(hyp, X, Y)
    n = size(X, 1); D = size(X, 2); sn = exp(hyp(D+2));
    K = seKernel(X, X, hyp); Ky = K + (sn^2 + 1e-6) * eye(n);
    try; L = chol(Ky, 'lower');
    catch; nlml = 1e10; return; end
    alpha = L' \ (L \ Y);
    nlml = 0.5 * (Y' * alpha) + sum(log(diag(L))) + 0.5 * n * log(2*pi);
end

function K = seKernel(X1, X2, hyp)
    D = size(X1, 2); ell = exp(hyp(1:D)); sf = exp(hyp(D+1));
    sq_dist = pdist2(X1 ./ ell', X2 ./ ell').^2; K = sf^2 * exp(-0.5 * sq_dist);
end

function [mu, sigma2] = gpPredict(model, X_test_norm)
    X_train=model.X_train; Y_train=model.Y_train; hyp=model.hyp;
    n_train=size(X_train,1); D=size(X_train,2); sn=exp(hyp(D+2));
    K=seKernel(X_train,X_train,hyp); Ky=K+(sn^2+1e-6)*eye(n_train);
    K_star_t=seKernel(X_test_norm,X_train,hyp); K_ss=seKernel(X_test_norm,X_test_norm,hyp);
    L=chol(Ky,'lower'); alpha=L'\(L\Y_train); mu=K_star_t*alpha; v=L\K_star_t';
    sigma2=diag(K_ss)-sum(v.^2,1)'; sigma2=max(sigma2,1e-9);
end
