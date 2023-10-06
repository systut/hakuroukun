function WriteToCsv(data, header)
    time = datestr(now,'yyyymmdd_HH_MM_SS');
    filename = "EXdata/2023/" + time + ".csv";
    
    write_data = data;
    table = array2table(write_data);
    table.Properties.VariableNames(1:4) = header;
    writetable(table,filename)
end