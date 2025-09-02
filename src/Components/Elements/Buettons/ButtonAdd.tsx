import { MenuItem } from "@mui/material";
import AddIcon from '@mui/icons-material/Add';

interface ButtonAddProps {
  onClick: () => void;
  description?: string;
}

const ButtonAdd = ({ onClick, description }: ButtonAddProps) => {
  return (
    <MenuItem title={description} onClick={onClick}>
      <AddIcon fontSize="small" style={{ marginRight: 2 }} color="info"/>
    </MenuItem>
  );
};

export default ButtonAdd;