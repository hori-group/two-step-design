function latter_name=ConvertForSave(formar_name)
% Function to convert numbers to string for save file names
% Convert '.' to 'p' and '-' to 'm 
    str_former = num2str(formar_name); 
    name_size = size(str_former,2);
    for ii = 1:name_size
        if str_former(ii) == '.'
            str_former(ii) = 'p';
        elseif str_former(ii) == '-'
            str_former(ii) = 'm';
        end
    end
    latter_name = str_former;
end