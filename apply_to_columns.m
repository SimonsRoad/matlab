function [ res ] = apply_to_columns(func, x)
  applyToGivenCol = @(func, matrix) @(col) func(matrix(:, col));
  newApplyToCols = @(func, matrix) arrayfun(applyToGivenCol(func, matrix), 1:size(matrix,2), 'UniformOutput', false)';
  takeAll = @(x) [x{:}];
  % same thing: 
  % takeAll = @(x) reshape([x{:}], size(x{1},1), size(x,1));
  genericApplyToRows = @(func, matrix) takeAll(newApplyToCols(func, matrix));

  res = genericApplyToRows(func, x);
end

